/** @file fc_app.c
 *  @brief 飞控主循环实现。
 *
 *  该模块负责加载配置、接收传感器帧、执行 PNG 制导并向环境程序发出
 *  控制指令。
 */
#include "fc/fc_app.h"

#include "common/config.h"
#include "common/logger.h"
#include "common/packet.h"
#include "common/protocol.h"
#include "common/status.h"
#include "fc/fc_health.h"
#include "fc/fc_modes.h"
#include "fc/fc_state.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define FC_PACKET_BUFFER_SIZE 1024u

typedef struct FcRuntimeConfig {
    /** @brief 环境进程 UDP 基础端口。 */
    unsigned int env_base_port;
    /** @brief 飞控进程 UDP 基础端口。 */
    unsigned int fc_base_port;
    /** @brief 环境目标 IPv4 地址。 */
    char host[64];
} FcRuntimeConfig;

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

    status = config_get_uint32(runtime, "network.environment_base_port", &out->env_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "network.flight_control_base_port", &out->fc_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "network.host", out->host, sizeof(out->host));
    return status;
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
    FcMode last_reported_mode = FC_POWER_ON;

    if (ctx == 0 || ctx->flight_control_path == 0 || ctx->runtime_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    memset(&fc_tree, 0, sizeof(fc_tree));
    memset(&runtime_tree, 0, sizeof(runtime_tree));
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
    status = load_runtime_config(&runtime_tree, &runtime_cfg);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: invalid runtime config: %s\n",
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

    fc_port = runtime_cfg.fc_base_port + (2u * ctx->instance_id);
    env_port = runtime_cfg.env_base_port + (2u * ctx->instance_id);
    status = bind_udp_socket(fc_port, &sock);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to bind UDP port %u: %s\n",
            fc_port,
            sim_status_to_string(status));
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
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }

    for (;;) {
        unsigned char buffer[FC_PACKET_BUFFER_SIZE];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t got = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        SensorFrame sensor;
        ControlCommand command;

        if (got < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            status = SIM_ERR_IO;
            break;
        }
        if ((size_t)got != SIM_SENSOR_PACKET_WIRE_SIZE) {
            (void)fprintf(stderr, "flight_control_sim: dropped packet size=%ld expected=%zu\n",
                (long)got,
                (size_t)SIM_SENSOR_PACKET_WIRE_SIZE);
            continue;
        }
        status = packet_decode_sensor_frame(buffer, (size_t)got, ctx->instance_id, &sensor);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "flight_control_sim: dropped packet status=%s\n",
                sim_status_to_string(status));
            continue;
        }
        status = flight_controller_step(&controller, &sensor, &command);
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
        from.sin_port = htons((uint16_t)env_port);
        status = send_control_command(sock, &from, ctx->instance_id, &command);
        if (status != SIM_OK) {
            break;
        }
    }

    if (sock >= 0) {
        (void)close(sock);
    }
    config_free(&fc_tree);
    config_free(&runtime_tree);
    (void)logger_info(&logger, "flight_control_sim stopped");
    return status == SIM_ERR_TIMEOUT ? SIM_OK : status;
}
