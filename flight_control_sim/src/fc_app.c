/** @file fc_app.c
 *  @brief 飞控主循环实现。
 *
 *  该模块负责加载配置、接收传感器帧、执行 PNG 制导并向环境程序发出
 *  控制指令。
 */
#define _POSIX_C_SOURCE 200809L

#include "fc/fc_app.h"

#include "common/build_info.h"
#include "common/config.h"
#include "common/crc32.h"
#include "common/logger.h"
#include "common/packet.h"
#include "common/protocol.h"
#include "common/realtime_pacer.h"
#include "common/status.h"
#include "fc/fc_health.h"
#include "fc/fc_internal_log.h"
#include "fc/fc_modes.h"
#include "fc/fc_state.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define FC_PACKET_BUFFER_SIZE 1024u
#define FC_MAX_CONFIG_WARNINGS 32u

typedef struct FcRuntimeConfig {
    /** @brief 环境进程 UDP 基础端口。 */
    unsigned int env_base_port;
    /** @brief 飞控进程 UDP 基础端口。 */
    unsigned int fc_base_port;
    /** @brief 环境目标 IPv4 地址。 */
    char host[64];
    /** @brief 本次任务输出根目录。 */
    char output_dir[256];
    /** @brief 是否输出飞控内部二进制日志。 */
    int binary_logs;
    /** @brief 日志刷新周期，单位帧。 */
    unsigned int flush_every_steps;
    /** @brief 环境进程墙钟运行模式，用于性能报告关联。 */
    char run_mode[32];
    /** @brief 环境命令同步策略，用于性能报告关联。 */
    char synchronization_mode[32];
} FcRuntimeConfig;

/** @brief 一类飞控墙钟耗时的固定内存统计。 */
typedef struct FcTimingAccumulator {
    size_t count;
    double total_s;
    double max_s;
} FcTimingAccumulator;

/** @brief 飞控进程墙钟性能和报文质量统计。 */
typedef struct FcPerformanceStats {
    double run_wall_time_s;
    double target_step_period_s;
    double startup_sensor_wait_time_s;
    size_t valid_sensor_frame_count;
    size_t dropped_packet_count;
    size_t receive_timeout_count;
    size_t graceful_stop_count;
    size_t protocol_minor_mismatch_count;
    FcTimingAccumulator sensor_receive_wait;
    FcTimingAccumulator receive_timeout_wait;
    FcTimingAccumulator controller_compute;
    FcTimingAccumulator internal_log_write;
    FcTimingAccumulator command_send;
} FcPerformanceStats;

/** @brief 飞控和共享运行时配置中的未识别字段集合。 */
typedef struct FcConfigWarningCollector {
    char source[32];
    char paths[FC_MAX_CONFIG_WARNINGS][256];
    size_t stored_count;
    size_t total_count;
} FcConfigWarningCollector;

/** @brief 收集飞控未知配置字段并输出启动 warning。 */
static void collect_unknown_config_key(
    const char *object_path,
    const char *key,
    void *user_data)
{
    FcConfigWarningCollector *collector = (FcConfigWarningCollector *)user_data;
    char full_path[256];

    if (collector == 0 || object_path == 0 || key == 0) {
        return;
    }
    (void)snprintf(
        full_path,
        sizeof(full_path),
        "%s.%s%s%s",
        collector->source,
        object_path[0] == '\0' ? "" : object_path,
        object_path[0] == '\0' ? "" : ".",
        key);
    (void)fprintf(stderr, "flight_control_sim: config warning: unrecognized field %s\n", full_path);
    if (collector->stored_count < FC_MAX_CONFIG_WARNINGS) {
        (void)snprintf(
            collector->paths[collector->stored_count],
            sizeof(collector->paths[collector->stored_count]),
            "%s",
            full_path);
        ++collector->stored_count;
    }
    ++collector->total_count;
}

/** @brief 审计飞控及共享运行时配置的直接对象键。 */
static SimStatus audit_fc_config_fields(
    const ConfigTree *fc,
    const ConfigTree *runtime,
    FcConfigWarningCollector *collector)
{
    static const char *const fc_root[] = {
        "schema_version", "scheduler", "guidance", "autopilot", "safety"
    };
    static const char *const scheduler_keys[] = { "base_rate_hz", "tasks" };
    static const char *const task_keys[] = { "name", "period_ticks" };
    static const char *const guidance_keys[] = {
        "type", "navigation_constant", "max_accel_mps2", "max_accel_rate_mps3"
    };
    static const char *const autopilot_keys[] = {
        "enable_attitude_loop", "enable_control_allocation", "max_attitude_cmd_rad",
        "max_body_rate_cmd_radps", "attitude_time_constant_s", "gyro_damping_gain",
        "fin_accel_effectiveness_mps2_per_rad", "max_fin_deflection_rad"
    };
    static const char *const safety_keys[] = {
        "sensor_timeout_s", "command_hold_s", "reject_nan", "reject_old_seq"
    };
    static const char *const runtime_root[] = {
        "schema_version", "campaign", "network", "logging", "tools", "instances"
    };
    static const char *const campaign_keys[] = {
        "campaign_id", "instance_count", "schedule", "run_mode", "synchronization_mode",
        "max_parallel_instances", "base_random_seed", "failure_strategy"
    };
    static const char *const network_keys[] = {
        "protocol_version_major", "protocol_version_minor", "environment_base_port",
        "flight_control_base_port", "host"
    };
    static const char *const logging_keys[] = {
        "output_dir", "instance_dir_template", "binary_logs", "event_log", "flush_every_steps"
    };
    const char *const *keys;
    const char *path;
    size_t key_count;
    size_t unknown_count = 0u;
    size_t task_count = 0u;
    size_t index;
    int exists = 0;
    SimStatus status;

#define CHECK(tree, source_name, object_path, allowed) \
    do { \
        (void)snprintf(collector->source, sizeof(collector->source), "%s", source_name); \
        keys = allowed; \
        path = object_path; \
        key_count = sizeof(allowed) / sizeof((allowed)[0]); \
        status = config_visit_unknown_keys( \
            tree, path, keys, key_count, collect_unknown_config_key, collector, &unknown_count); \
        if (status != SIM_OK) { \
            return status; \
        } \
    } while (0)
    CHECK(fc, "flight_control", "", fc_root);
    CHECK(fc, "flight_control", "scheduler", scheduler_keys);
    CHECK(fc, "flight_control", "guidance", guidance_keys);
    status = config_path_exists(fc, "autopilot", &exists);
    if (status != SIM_OK) {
        return status;
    }
    if (exists != 0) {
        CHECK(fc, "flight_control", "autopilot", autopilot_keys);
    }
    CHECK(fc, "flight_control", "safety", safety_keys);
    status = config_get_array_count(fc, "scheduler.tasks", &task_count);
    if (status != SIM_OK && status != SIM_ERR_CONFIG) {
        return status;
    }
    for (index = 0u; status == SIM_OK && index < task_count; ++index) {
        char task_path[64];

        (void)snprintf(task_path, sizeof(task_path), "scheduler.tasks[%zu]", index);
        (void)snprintf(collector->source, sizeof(collector->source), "flight_control");
        status = config_visit_unknown_keys(
            fc,
            task_path,
            task_keys,
            sizeof(task_keys) / sizeof(task_keys[0]),
            collect_unknown_config_key,
            collector,
            &unknown_count);
        if (status != SIM_OK) {
            return status;
        }
    }
    CHECK(runtime, "runtime", "", runtime_root);
    CHECK(runtime, "runtime", "campaign", campaign_keys);
    CHECK(runtime, "runtime", "network", network_keys);
    CHECK(runtime, "runtime", "logging", logging_keys);
#undef CHECK
    return SIM_OK;
}

/** @brief 记录一次有限非负飞控墙钟耗时。 */
static void timing_accumulate(FcTimingAccumulator *stats, double duration_s)
{
    if (stats == 0 || !isfinite(duration_s) || duration_s < 0.0) {
        return;
    }
    ++stats->count;
    stats->total_s += duration_s;
    if (duration_s > stats->max_s) {
        stats->max_s = duration_s;
    }
}

/** @brief 完成一次飞控墙钟计时。 */
static SimStatus timing_finish(double start_s, FcTimingAccumulator *stats)
{
    double end_s;
    SimStatus status = monotonic_time_now(&end_s);

    if (status == SIM_OK) {
        timing_accumulate(stats, end_s - start_s);
    }
    return status;
}

/** @brief 从飞控配置中读取安全保护参数。 */
static SimStatus load_safety_config(const ConfigTree *config, FcSafetyConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "safety.sensor_timeout_s", &out->sensor_timeout_s);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, "safety.command_hold_s", &out->command_hold_s);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_bool(config, "safety.reject_nan", &out->reject_nan);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_bool(config, "safety.reject_old_seq", &out->reject_old_seq);
    if (status != SIM_OK) {
        return status;
    }
    out->max_consecutive_bad_frames = 3u;
    return SIM_OK;
}

/** @brief 检查配置任务表是否包含指定任务名。 */
static int scheduler_config_has_task(const FlightControllerConfig *config, const char *name)
{
    uint32_t index;

    if (config == 0 || name == 0) {
        return 0;
    }
    for (index = 0u; index < config->scheduler_task_count; ++index) {
        if (strcmp(config->scheduler_tasks[index].name, name) == 0) {
            return 1;
        }
    }
    return 0;
}

/** @brief 从飞控配置中读取调度器基准频率和任务周期表。 */
static SimStatus load_scheduler_config(const ConfigTree *config, FlightControllerConfig *out)
{
    size_t task_count = 0u;
    size_t index;
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "scheduler.base_rate_hz", &out->scheduler_base_rate_hz);
    if (status != SIM_OK) {
        return status;
    }
    if (config_get_array_count(config, "scheduler.tasks", &task_count) != SIM_OK) {
        out->scheduler_task_count = 0u;
        return SIM_OK;
    }
    if (task_count > FC_SCHEDULER_MAX_TASKS) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    out->scheduler_task_count = (uint32_t)task_count;
    for (index = 0u; index < task_count; ++index) {
        char path[96];
        unsigned int period_ticks = 0u;
        FcTask *task = &out->scheduler_tasks[index];

        (void)snprintf(path, sizeof(path), "scheduler.tasks[%u].name", (unsigned int)index);
        status = config_get_string(config, path, task->name, sizeof(task->name));
        if (status != SIM_OK) {
            return status;
        }
        (void)snprintf(path, sizeof(path), "scheduler.tasks[%u].period_ticks", (unsigned int)index);
        status = config_get_uint32(config, path, &period_ticks);
        if (status != SIM_OK || period_ticks == 0u) {
            return status == SIM_OK ? SIM_ERR_OUT_OF_RANGE : status;
        }
        task->period_ticks = period_ticks;
    }
    if (!scheduler_config_has_task(out, "receive") ||
        !scheduler_config_has_task(out, "navigation") ||
        !scheduler_config_has_task(out, "guidance") ||
        !scheduler_config_has_task(out, "controller") ||
        !scheduler_config_has_task(out, "safety")) {
        return SIM_ERR_CONFIG;
    }
    return SIM_OK;
}

/** @brief 从运行时配置中读取网络参数。 */
static SimStatus load_runtime_config(const ConfigTree *runtime, FcRuntimeConfig *out)
{
    SimStatus status;

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));

    status = config_get_uint32(runtime, "network.environment_base_port", &out->env_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "network.flight_control_base_port", &out->fc_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "network.host", out->host, sizeof(out->host));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "logging.output_dir", out->output_dir, sizeof(out->output_dir));
    if (status != SIM_OK) {
        out->output_dir[0] = '\0';
    }
    status = config_get_bool(runtime, "logging.binary_logs", &out->binary_logs);
    if (status != SIM_OK) {
        out->binary_logs = 1;
    }
    status = config_get_uint32(runtime, "logging.flush_every_steps", &out->flush_every_steps);
    if (status != SIM_OK || out->flush_every_steps == 0u) {
        out->flush_every_steps = 100u;
    }
    {
        int run_mode_exists = 0;

        status = config_path_exists(runtime, "campaign.run_mode", &run_mode_exists);
        if (status != SIM_OK) {
            return status;
        }
        if (run_mode_exists == 0) {
            (void)snprintf(out->run_mode, sizeof(out->run_mode), "SIL_FAST");
        } else {
            status = config_get_string(
                runtime,
                "campaign.run_mode",
                out->run_mode,
                sizeof(out->run_mode));
            if (status != SIM_OK) {
                return status;
            }
            if (strcmp(out->run_mode, "SIL_FAST") != 0 &&
                strcmp(out->run_mode, "SIL_REALTIME") != 0) {
                return SIM_ERR_CONFIG;
            }
        }
    }
    {
        int synchronization_mode_exists = 0;

        status = config_path_exists(
            runtime,
            "campaign.synchronization_mode",
            &synchronization_mode_exists);
        if (status != SIM_OK) {
            return status;
        }
        if (synchronization_mode_exists == 0) {
            (void)snprintf(
                out->synchronization_mode,
                sizeof(out->synchronization_mode),
                "LOCKSTEP");
        } else {
            status = config_get_string(
                runtime,
                "campaign.synchronization_mode",
                out->synchronization_mode,
                sizeof(out->synchronization_mode));
            if (status != SIM_OK) {
                return status;
            }
            if (strcmp(out->synchronization_mode, "LOCKSTEP") != 0 &&
                strcmp(out->synchronization_mode, "FREE_RUNNING") != 0) {
                return SIM_ERR_CONFIG;
            }
        }
    }
    return SIM_OK;
}

/** @brief 从飞控配置中读取 PNG 参数。 */
static SimStatus load_guidance_config(const ConfigTree *config, GuidancePngConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "guidance.navigation_constant", &out->navigation_constant);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, "guidance.max_accel_mps2", &out->max_accel_mps2);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, "guidance.max_accel_rate_mps3", &out->max_accel_rate_mps3);
    return status;
}

/** @brief 从飞控配置中读取自动驾驶仪参数，缺失时使用保守默认值。 */
static SimStatus load_autopilot_config(const ConfigTree *config, AutopilotConfig *out)
{
    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    out->enable_attitude_loop = 1;
    out->enable_control_allocation = 1;
    out->max_attitude_cmd_rad = 0.35;
    out->max_body_rate_cmd_radps = 1.0;
    out->attitude_time_constant_s = 0.25;
    out->gyro_damping_gain = 0.2;
    out->fin_accel_effectiveness_mps2_per_rad = 150.0;
    out->max_fin_deflection_rad = 0.35;
    (void)config_get_bool(config, "autopilot.enable_attitude_loop", &out->enable_attitude_loop);
    (void)config_get_bool(config, "autopilot.enable_control_allocation", &out->enable_control_allocation);
    (void)config_get_double(config, "autopilot.max_attitude_cmd_rad", &out->max_attitude_cmd_rad);
    (void)config_get_double(config, "autopilot.max_body_rate_cmd_radps", &out->max_body_rate_cmd_radps);
    (void)config_get_double(config, "autopilot.attitude_time_constant_s", &out->attitude_time_constant_s);
    (void)config_get_double(config, "autopilot.gyro_damping_gain", &out->gyro_damping_gain);
    (void)config_get_double(
        config,
        "autopilot.fin_accel_effectiveness_mps2_per_rad",
        &out->fin_accel_effectiveness_mps2_per_rad);
    (void)config_get_double(config, "autopilot.max_fin_deflection_rad", &out->max_fin_deflection_rad);
    return SIM_OK;
}

/** @brief 绑定 UDP 监听套接字。 */
static SimStatus bind_udp_socket(unsigned int port, int *sock_out)
{
    int sock;
    struct sockaddr_in addr;
    struct timeval timeout;

    if (sock_out == 0 || port > 65535u) {
        return SIM_ERR_INVALID_ARG;
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return SIM_ERR_IO;
    }

    {
        int reuse = 1;
        (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    }
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        (void)close(sock);
        return SIM_ERR_IO;
    }

    *sock_out = sock;
    return SIM_OK;
}

/** @brief 序列化并发送控制指令。 */
static SimStatus send_control_command(
    int sock,
    const struct sockaddr_in *peer,
    uint32_t instance_id,
    const ControlCommand *command)
{
    unsigned char buffer[SIM_CONTROL_PACKET_WIRE_SIZE];
    size_t packet_size;
    SimStatus status;
    ssize_t sent;

    if (peer == 0 || command == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    status = packet_encode_control_command(
        instance_id,
        command,
        buffer,
        sizeof(buffer),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
    if (sent != (ssize_t)packet_size) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 向实例管理器发送应用层就绪心跳。 */
static SimStatus send_ready_heartbeat(
    int sock,
    const char *host,
    unsigned int ready_port,
    uint32_t instance_id)
{
    unsigned char buffer[SIM_HEARTBEAT_PACKET_WIRE_SIZE];
    struct sockaddr_in peer;
    size_t packet_size = 0u;
    ssize_t sent;
    SimStatus status;

    if (ready_port == 0u) {
        return SIM_OK;
    }
    if (host == 0 || ready_port > 65535u) {
        return SIM_ERR_INVALID_ARG;
    }
    status = packet_encode_heartbeat(instance_id, 1u, 0.0, buffer, sizeof(buffer), &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    (void)memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)ready_port);
    if (inet_pton(AF_INET, host, &peer.sin_addr) != 1) {
        return SIM_ERR_CONFIG;
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)&peer, sizeof(peer));
    return sent == (ssize_t)packet_size ? SIM_OK : SIM_ERR_IO;
}

/** @brief 创建飞控内部日志实例目录。 */
static SimStatus make_fc_instance_dir(
    const char *base_dir,
    uint32_t instance_id,
    char *instance_dir,
    size_t instance_dir_size)
{
    int written;

    if (base_dir == 0 || base_dir[0] == '\0' || instance_dir == 0 || instance_dir_size == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)mkdir("runs", 0777);
    if (mkdir(base_dir, 0777) != 0 && errno != EEXIST) {
        return SIM_ERR_IO;
    }
    written = snprintf(instance_dir, instance_dir_size, "%s/instance_%04u", base_dir, instance_id);
    if (written < 0 || (size_t)written >= instance_dir_size) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (mkdir(instance_dir, 0777) != 0 && errno != EEXIST) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 将已校验配置树按输入原始字节写入实例快照。 */
static SimStatus write_config_snapshot(
    const ConfigTree *tree,
    const char *instance_dir,
    const char *name)
{
    char path[1024];
    FILE *file;

    if (tree == 0 || tree->data == 0 || instance_dir == 0 || name == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)snprintf(path, sizeof(path), "%s/%s", instance_dir, name);
    file = fopen(path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fwrite(tree->data, 1u, tree->size, file) != tree->size) {
        (void)fclose(file);
        return SIM_ERR_IO;
    }
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 格式化当前 UTC 墙钟，仅写 provenance，不参与控制计算。 */
static void format_wall_clock_utc(char *out, size_t out_size)
{
    time_t now;
    struct tm utc;

    if (out == 0 || out_size == 0u) {
        return;
    }
    now = time(0);
    if (now == (time_t)-1 || gmtime_r(&now, &utc) == 0 ||
        strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &utc) == 0u) {
        (void)snprintf(out, out_size, "unknown");
    }
}

/** @brief 写出飞控配置、软件身份和日志路径的独立运行清单。 */
static SimStatus write_fc_run_manifest(
    const FcContext *ctx,
    const FcRuntimeConfig *runtime,
    const ConfigTree *fc_tree,
    const ConfigTree *runtime_tree,
    const FcConfigWarningCollector *config_warnings)
{
    char instance_dir[512];
    char path[1024];
    char campaign_id[128] = "unknown";
    char start_time_wall_clock[32];
    FILE *file;
    SimStatus status;

    if (ctx == 0 || runtime == 0 || fc_tree == 0 || runtime_tree == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = make_fc_instance_dir(
        runtime->output_dir,
        ctx->instance_id,
        instance_dir,
        sizeof(instance_dir));
    if (status != SIM_OK) {
        return status;
    }
    status = write_config_snapshot(
        fc_tree,
        instance_dir,
        "config_snapshot_flight_control.json");
    if (status == SIM_OK) {
        status = write_config_snapshot(
            runtime_tree,
            instance_dir,
            "fc_config_snapshot_runtime.json");
    }
    if (status != SIM_OK) {
        return status;
    }
    (void)config_get_string(
        runtime_tree,
        "campaign.campaign_id",
        campaign_id,
        sizeof(campaign_id));
    format_wall_clock_utc(start_time_wall_clock, sizeof(start_time_wall_clock));
    (void)snprintf(path, sizeof(path), "%s/fc_run_manifest.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"instance_id\": %u,\n", ctx->instance_id);
    (void)fprintf(file, "  \"campaign_id\": \"%s\",\n", campaign_id);
    (void)fprintf(
        file,
        "  \"program_version\": \"flight_control_sim %d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(file, "  \"git_commit\": \"%s\",\n", MISSILE_SIM_GIT_COMMIT);
    (void)fprintf(
        file,
        "  \"git_worktree_dirty\": %s,\n",
        MISSILE_SIM_GIT_DIRTY != 0 ? "true" : "false");
    (void)fprintf(file, "  \"build_time\": \"%s\",\n", MISSILE_SIM_BUILD_TIME);
    (void)fprintf(file, "  \"compiler\": \"%s\",\n", MISSILE_SIM_COMPILER);
    (void)fprintf(file, "  \"start_time_wall_clock\": \"%s\",\n", start_time_wall_clock);
    (void)fprintf(
        file,
        "  \"protocol_version\": \"%d.%d\",\n",
        MISSILE_SIM_PROTOCOL_VERSION_MAJOR,
        MISSILE_SIM_PROTOCOL_VERSION_MINOR);
    (void)fprintf(file, "  \"flight_control_path\": \"%s\",\n", ctx->flight_control_path);
    (void)fprintf(file, "  \"runtime_path\": \"%s\",\n", ctx->runtime_path);
    (void)fprintf(
        file,
        "  \"config_crc32\": { \"flight_control\": \"0x%08x\", "
        "\"runtime\": \"0x%08x\" },\n",
        crc32_compute(fc_tree->data, fc_tree->size),
        crc32_compute(runtime_tree->data, runtime_tree->size));
    (void)fprintf(file, "  \"config_snapshots\": {\n");
    (void)fprintf(
        file,
        "    \"flight_control\": \"%s/config_snapshot_flight_control.json\",\n",
        instance_dir);
    (void)fprintf(
        file,
        "    \"runtime\": \"%s/fc_config_snapshot_runtime.json\"\n",
        instance_dir);
    (void)fprintf(file, "  },\n");
    (void)fprintf(
        file,
        "  \"unrecognized_config_field_count\": %lu,\n",
        config_warnings != 0 ? (unsigned long)config_warnings->total_count : 0ul);
    (void)fprintf(file, "  \"unrecognized_config_fields\": [");
    if (config_warnings != 0) {
        size_t warning_index;

        for (warning_index = 0u;
             warning_index < config_warnings->stored_count;
             ++warning_index) {
            (void)fprintf(
                file,
                "%s\"%s\"",
                warning_index == 0u ? "" : ", ",
                config_warnings->paths[warning_index]);
        }
    }
    (void)fprintf(file, "],\n");
    (void)fprintf(file, "  \"run_mode\": \"%s\",\n", runtime->run_mode);
    (void)fprintf(
        file,
        "  \"synchronization_mode\": \"%s\",\n",
        runtime->synchronization_mode);
    (void)fprintf(file, "  \"log_files\": {\n");
    (void)fprintf(file, "    \"internal\": \"%s/fc_internal_log.bin\",\n", instance_dir);
    (void)fprintf(file, "    \"performance\": \"%s/fc_performance.json\"\n", instance_dir);
    (void)fprintf(file, "  }\n");
    (void)fprintf(file, "}\n");
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 打开飞控内部持久化日志。 */
static FILE *open_internal_log(const FcRuntimeConfig *runtime, uint32_t instance_id)
{
    char instance_dir[512];
    char path[1024];

    if (runtime == 0 || runtime->binary_logs == 0 || runtime->output_dir[0] == '\0') {
        return 0;
    }
    if (make_fc_instance_dir(runtime->output_dir, instance_id, instance_dir, sizeof(instance_dir)) != SIM_OK) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/fc_internal_log.bin", instance_dir);
    return fopen(path, "wb");
}

/** @brief 写入一帧飞控内部状态日志。 */
static SimStatus write_internal_log(FILE *file, const ControlCommand *command)
{
    unsigned char record[FC_INTERNAL_LOG_WIRE_SIZE];
    SimStatus status;

    if (file == 0) {
        return SIM_OK;
    }
    if (command == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = fc_internal_log_encode(command, record, sizeof(record));
    if (status != SIM_OK) {
        return status;
    }
    return fwrite(record, 1u, sizeof(record), file) == sizeof(record) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 写出不参与控制计算和确定性回归的飞控墙钟性能报告。 */
static SimStatus write_performance_report(
    const FcRuntimeConfig *runtime,
    uint32_t instance_id,
    const FcPerformanceStats *stats,
    const FcConfigWarningCollector *config_warnings)
{
    char instance_dir[512];
    char path[1024];
    FILE *file;

    if (runtime == 0 || stats == 0 || runtime->output_dir[0] == '\0') {
        return SIM_ERR_INVALID_ARG;
    }
    if (make_fc_instance_dir(
            runtime->output_dir,
            instance_id,
            instance_dir,
            sizeof(instance_dir)) != SIM_OK) {
        return SIM_ERR_IO;
    }
    (void)snprintf(path, sizeof(path), "%s/fc_performance.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"scope\": \"wall_clock_diagnostics_not_control_input\",\n");
    (void)fprintf(
        file,
        "  \"unrecognized_config_field_count\": %lu,\n",
        config_warnings != 0 ? (unsigned long)config_warnings->total_count : 0ul);
    (void)fprintf(file, "  \"unrecognized_config_fields\": [");
    if (config_warnings != 0) {
        size_t warning_index;

        for (warning_index = 0u;
             warning_index < config_warnings->stored_count;
             ++warning_index) {
            (void)fprintf(
                file,
                "%s\"%s\"",
                warning_index == 0u ? "" : ", ",
                config_warnings->paths[warning_index]);
        }
    }
    (void)fprintf(file, "],\n");
    (void)fprintf(file, "  \"run_mode\": \"%s\",\n", runtime->run_mode);
    (void)fprintf(
        file,
        "  \"synchronization_mode\": \"%s\",\n",
        runtime->synchronization_mode);
    (void)fprintf(file, "  \"target_step_period_s\": %.9f,\n", stats->target_step_period_s);
    (void)fprintf(file, "  \"run_wall_time_s\": %.9f,\n", stats->run_wall_time_s);
    (void)fprintf(
        file,
        "  \"startup_sensor_wait_time_s\": %.12f,\n",
        stats->startup_sensor_wait_time_s);
    (void)fprintf(
        file,
        "  \"valid_sensor_frame_count\": %lu,\n",
        (unsigned long)stats->valid_sensor_frame_count);
    (void)fprintf(
        file,
        "  \"dropped_packet_count\": %lu,\n",
        (unsigned long)stats->dropped_packet_count);
    (void)fprintf(
        file,
        "  \"receive_timeout_count\": %lu,\n",
        (unsigned long)stats->receive_timeout_count);
    (void)fprintf(
        file,
        "  \"graceful_stop_count\": %lu,\n",
        (unsigned long)stats->graceful_stop_count);
    (void)fprintf(
        file,
        "  \"protocol_minor_mismatch_count\": %lu,\n",
        (unsigned long)stats->protocol_minor_mismatch_count);
    (void)fprintf(
        file,
        "  \"mean_sensor_receive_wait_time_s\": %.12f,\n",
        stats->sensor_receive_wait.count > 0u ?
            stats->sensor_receive_wait.total_s / (double)stats->sensor_receive_wait.count : 0.0);
    (void)fprintf(
        file,
        "  \"max_sensor_receive_wait_time_s\": %.12f,\n",
        stats->sensor_receive_wait.max_s);
    (void)fprintf(
        file,
        "  \"mean_receive_timeout_wait_time_s\": %.12f,\n",
        stats->receive_timeout_wait.count > 0u ?
            stats->receive_timeout_wait.total_s / (double)stats->receive_timeout_wait.count : 0.0);
    (void)fprintf(
        file,
        "  \"max_receive_timeout_wait_time_s\": %.12f,\n",
        stats->receive_timeout_wait.max_s);
    (void)fprintf(
        file,
        "  \"controller_compute_sample_count\": %lu,\n",
        (unsigned long)stats->controller_compute.count);
    (void)fprintf(
        file,
        "  \"mean_controller_compute_time_s\": %.12f,\n",
        stats->controller_compute.count > 0u ?
            stats->controller_compute.total_s / (double)stats->controller_compute.count : 0.0);
    (void)fprintf(
        file,
        "  \"max_controller_compute_time_s\": %.12f,\n",
        stats->controller_compute.max_s);
    (void)fprintf(
        file,
        "  \"mean_internal_log_write_time_s\": %.12f,\n",
        stats->internal_log_write.count > 0u ?
            stats->internal_log_write.total_s / (double)stats->internal_log_write.count : 0.0);
    (void)fprintf(
        file,
        "  \"max_internal_log_write_time_s\": %.12f,\n",
        stats->internal_log_write.max_s);
    (void)fprintf(
        file,
        "  \"mean_command_send_time_s\": %.12f,\n",
        stats->command_send.count > 0u ?
            stats->command_send.total_s / (double)stats->command_send.count : 0.0);
    (void)fprintf(
        file,
        "  \"max_command_send_time_s\": %.12f,\n",
        stats->command_send.max_s);
    (void)fprintf(
        file,
        "  \"controller_realtime_margin_s\": %.12f\n",
        stats->target_step_period_s - stats->controller_compute.max_s);
    (void)fprintf(file, "}\n");
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 运行飞控仿真主循环。 */
SimStatus fc_app_run(const FcContext *ctx)
{
    ConfigTree fc_tree;
    ConfigTree runtime_tree;
    FcRuntimeConfig runtime_cfg;
    FlightControllerConfig controller_cfg;
    FlightController controller;
    Logger logger;
    SimStatus status;
    int sock = -1;
    unsigned int fc_port;
    unsigned int env_port;
    struct sockaddr_in env_addr;
    FcMode last_reported_mode = FC_POWER_ON;
    FILE *internal_log = 0;
    uint32_t logged_frames = 0u;
    FcPerformanceStats performance_stats;
    FcConfigWarningCollector config_warnings;
    double run_wall_start_s = 0.0;
    int protocol_minor_warning_written = 0;

    if (ctx == 0 || ctx->flight_control_path == 0 || ctx->runtime_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    memset(&fc_tree, 0, sizeof(fc_tree));
    memset(&runtime_tree, 0, sizeof(runtime_tree));
    memset(&performance_stats, 0, sizeof(performance_stats));
    memset(&config_warnings, 0, sizeof(config_warnings));
    status = logger_open_stdout(&logger);
    if (status != SIM_OK) {
        return status;
    }
    status = config_load_file(ctx->flight_control_path, &fc_tree);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to load %s: %s\n",
            ctx->flight_control_path,
            sim_status_to_string(status));
        return status;
    }
    status = config_load_file(ctx->runtime_path, &runtime_tree);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to load %s: %s\n",
            ctx->runtime_path,
            sim_status_to_string(status));
        config_free(&fc_tree);
        return status;
    }
    status = config_validate_schema(&fc_tree, 1u);
    if (status == SIM_OK) {
        status = config_require_section(&fc_tree, "guidance");
    }
    if (status == SIM_OK) {
        status = config_require_section(&fc_tree, "safety");
    }
    if (status == SIM_OK) {
        status = config_validate_schema(&runtime_tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime_tree, "network");
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: schema validation failed: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = audit_fc_config_fields(&fc_tree, &runtime_tree, &config_warnings);
    if (status != SIM_OK) {
        (void)fprintf(
            stderr,
            "flight_control_sim: config field audit failed: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = load_runtime_config(&runtime_tree, &runtime_cfg);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: invalid runtime config: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = write_fc_run_manifest(
        ctx,
        &runtime_cfg,
        &fc_tree,
        &runtime_tree,
        &config_warnings);
    if (status != SIM_OK) {
        (void)fprintf(
            stderr,
            "flight_control_sim: failed to write provenance: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    memset(&controller_cfg, 0, sizeof(controller_cfg));
    status = load_guidance_config(&fc_tree, &controller_cfg.guidance);
    if (status == SIM_OK) {
        status = load_autopilot_config(&fc_tree, &controller_cfg.autopilot);
    }
    if (status == SIM_OK) {
        status = load_safety_config(&fc_tree, &controller_cfg.safety);
    }
    if (status == SIM_OK) {
        status = load_scheduler_config(&fc_tree, &controller_cfg);
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: invalid flight-control config: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = flight_controller_init(&controller, &controller_cfg);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: controller init failed: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    internal_log = open_internal_log(&runtime_cfg, ctx->instance_id);
    if (runtime_cfg.binary_logs != 0 && internal_log == 0) {
        (void)fprintf(stderr, "flight_control_sim: failed to open internal log\n");
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return SIM_ERR_IO;
    }

    fc_port = runtime_cfg.fc_base_port + (2u * ctx->instance_id);
    env_port = runtime_cfg.env_base_port + (2u * ctx->instance_id);
    (void)memset(&env_addr, 0, sizeof(env_addr));
    env_addr.sin_family = AF_INET;
    env_addr.sin_port = htons((uint16_t)env_port);
    if (inet_pton(AF_INET, runtime_cfg.host, &env_addr.sin_addr) != 1) {
        if (internal_log != 0) {
            (void)fclose(internal_log);
        }
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return SIM_ERR_CONFIG;
    }
    status = bind_udp_socket(fc_port, &sock);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to bind UDP port %u: %s\n",
            fc_port,
            sim_status_to_string(status));
        if (internal_log != 0) {
            (void)fclose(internal_log);
        }
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }

    (void)logger_info(&logger, "flight_control_sim UDP loop started");
    (void)printf("instance_id=%u fc_port=%u env_port=%u\n", ctx->instance_id, fc_port, env_port);
    status = send_ready_heartbeat(sock, runtime_cfg.host, ctx->ready_port, ctx->instance_id);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to send ready heartbeat: %s\n",
            sim_status_to_string(status));
        (void)close(sock);
        if (internal_log != 0) {
            (void)fclose(internal_log);
        }
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = monotonic_time_now(&run_wall_start_s);
    if (status != SIM_OK) {
        (void)close(sock);
        if (internal_log != 0) {
            (void)fclose(internal_log);
        }
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }

    for (;;) {
        unsigned char buffer[FC_PACKET_BUFFER_SIZE];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t got;
        int receive_errno = 0;
        SensorFrame sensor;
        ControlCommand command;
        double receive_start_s = 0.0;
        double receive_end_s = 0.0;
        double receive_duration_s = 0.0;

        status = monotonic_time_now(&receive_start_s);
        if (status != SIM_OK) {
            break;
        }
        got = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        if (got < 0) {
            receive_errno = errno;
        }
        status = monotonic_time_now(&receive_end_s);
        if (status != SIM_OK) {
            break;
        }
        receive_duration_s = receive_end_s - receive_start_s;
        if (!isfinite(receive_duration_s) || receive_duration_s < 0.0) {
            status = SIM_ERR_NUMERIC;
            break;
        }
        if (got < 0) {
            if (receive_errno == EAGAIN || receive_errno == EWOULDBLOCK) {
                ++performance_stats.receive_timeout_count;
                timing_accumulate(
                    &performance_stats.receive_timeout_wait,
                    receive_duration_s);
                break;
            }
            status = SIM_ERR_IO;
            break;
        }
        if (from.sin_family != env_addr.sin_family ||
            from.sin_port != env_addr.sin_port ||
            from.sin_addr.s_addr != env_addr.sin_addr.s_addr) {
            ++performance_stats.dropped_packet_count;
            (void)fprintf(stderr, "flight_control_sim: dropped packet from unexpected peer\n");
            continue;
        }
        {
            PacketHeader header;

            status = packet_peek_header(buffer, (size_t)got, &header);
            if (status != SIM_OK) {
                ++performance_stats.dropped_packet_count;
                (void)fprintf(stderr, "flight_control_sim: dropped invalid packet header\n");
                continue;
            }
            if (header.version_minor != MISSILE_SIM_PROTOCOL_VERSION_MINOR) {
                ++performance_stats.protocol_minor_mismatch_count;
                if (protocol_minor_warning_written == 0) {
                    (void)fprintf(
                        stderr,
                        "flight_control_sim: protocol minor version %u differs from supported %u; "
                        "using compatible v1 fields\n",
                        (unsigned int)header.version_minor,
                        (unsigned int)MISSILE_SIM_PROTOCOL_VERSION_MINOR);
                    protocol_minor_warning_written = 1;
                }
            }
        }
        if ((size_t)got == SIM_SIM_CONTROL_PACKET_WIRE_SIZE) {
            PacketHeader control_header;
            SimControlAction action = (SimControlAction)0;

            status = packet_decode_sim_control(
                buffer,
                (size_t)got,
                ctx->instance_id,
                &control_header,
                &action);
            if (status == SIM_OK && action == SIM_CONTROL_STOP) {
                ++performance_stats.graceful_stop_count;
                break;
            }
            ++performance_stats.dropped_packet_count;
            (void)fprintf(stderr, "flight_control_sim: dropped invalid simulation control packet\n");
            continue;
        }
        if ((size_t)got != SIM_SENSOR_PACKET_WIRE_SIZE) {
            ++performance_stats.dropped_packet_count;
            (void)fprintf(stderr, "flight_control_sim: dropped packet size=%ld expected=%zu\n",
                (long)got,
                (size_t)SIM_SENSOR_PACKET_WIRE_SIZE);
            continue;
        }
        status = packet_decode_sensor_frame(buffer, (size_t)got, ctx->instance_id, &sensor);
        if (status != SIM_OK) {
            ++performance_stats.dropped_packet_count;
            (void)fprintf(stderr, "flight_control_sim: dropped packet status=%s\n",
                sim_status_to_string(status));
            continue;
        }
        if (performance_stats.valid_sensor_frame_count == 0u) {
            performance_stats.startup_sensor_wait_time_s = receive_duration_s;
        } else {
            timing_accumulate(&performance_stats.sensor_receive_wait, receive_duration_s);
        }
        ++performance_stats.valid_sensor_frame_count;
        if (performance_stats.target_step_period_s == 0.0) {
            performance_stats.target_step_period_s = sensor.dt;
        }
        {
            double controller_start_s = 0.0;

            status = monotonic_time_now(&controller_start_s);
            if (status == SIM_OK) {
                status = flight_controller_step(&controller, &sensor, &command);
            }
            if (status == SIM_OK) {
                status = timing_finish(controller_start_s, &performance_stats.controller_compute);
            }
        }
        if (status != SIM_OK) {
            (void)fprintf(stderr, "flight_control_sim: controller step failed seq=%u status=%s\n",
                sensor.seq,
                sim_status_to_string(status));
            break;
        }
        if (controller.mode != last_reported_mode) {
            (void)fprintf(stdout, "flight_control_sim: mode %s -> %s at t=%.6f seq=%u\n",
                fc_mode_to_string(last_reported_mode),
                fc_mode_to_string(controller.mode),
                sensor.sim_time,
                sensor.seq);
            last_reported_mode = controller.mode;
        }
        if (command.command_status != FC_COMMAND_STATUS_OK) {
            (void)fprintf(stdout, "flight_control_sim: protection seq=%u mode=%s status=0x%08x\n",
                command.seq,
                fc_mode_to_string((FcMode)command.command_mode),
                command.command_status);
        }
        {
            double log_start_s = 0.0;

            status = monotonic_time_now(&log_start_s);
            if (status == SIM_OK) {
                status = write_internal_log(internal_log, &command);
            }
            if (status == SIM_OK) {
                ++logged_frames;
                if (internal_log != 0 &&
                    runtime_cfg.flush_every_steps > 0u &&
                    (logged_frames % runtime_cfg.flush_every_steps) == 0u) {
                    if (fflush(internal_log) != 0) {
                        status = SIM_ERR_IO;
                    }
                }
            }
            if (status == SIM_OK) {
                status = timing_finish(log_start_s, &performance_stats.internal_log_write);
            }
        }
        if (status != SIM_OK) {
            (void)fprintf(stderr, "flight_control_sim: internal log failed: %s\n",
                sim_status_to_string(status));
            break;
        }
        {
            double send_start_s = 0.0;

            status = monotonic_time_now(&send_start_s);
            if (status == SIM_OK) {
                status = send_control_command(sock, &env_addr, ctx->instance_id, &command);
            }
            if (status == SIM_OK) {
                status = timing_finish(send_start_s, &performance_stats.command_send);
            }
        }
        if (status != SIM_OK) {
            break;
        }
    }

    {
        double run_wall_end_s = 0.0;

        if (monotonic_time_now(&run_wall_end_s) == SIM_OK && run_wall_end_s >= run_wall_start_s) {
            performance_stats.run_wall_time_s = run_wall_end_s - run_wall_start_s;
        }
    }

    if (sock >= 0) {
        (void)close(sock);
    }
    if (internal_log != 0) {
        (void)fclose(internal_log);
    }
    {
        SimStatus report_status = write_performance_report(
            &runtime_cfg,
            ctx->instance_id,
            &performance_stats,
            &config_warnings);

        if (status == SIM_OK && report_status != SIM_OK) {
            status = report_status;
        }
    }
    config_free(&fc_tree);
    config_free(&runtime_tree);
    (void)logger_info(&logger, "flight_control_sim stopped");
    return status == SIM_ERR_TIMEOUT ? SIM_OK : status;
}
