/** @file main.c
 *  @brief 多实例编排管理器实现。
 *
 *  该工具根据 runtime.json 的 instances[] 启动成对的环境与飞控进程，
 *  每个实例拥有独立 instance_id、UDP 端口、输出目录、配置路径和随机种子。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/config.h"
#include "common/status.h"

#include <arpa/inet.h>
#include <errno.h>
#include <float.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RUNTIME_CONFIG "configs/baseline/runtime.json"
#define DEFAULT_SCENARIO_CONFIG "configs/baseline/scenario.json"
#define DEFAULT_FC_CONFIG "configs/baseline/flight_control.json"
#define DEFAULT_FAULTS_CONFIG "configs/baseline/faults.json"
#define ENVIRONMENT_PROGRAM "./build/environment_sim/environment_sim"
#define FLIGHT_CONTROL_PROGRAM "./build/flight_control_sim/flight_control_sim"
#define MAX_INSTANCES 128u
#define MANAGER_PATH_SIZE 256u

typedef enum ManagerSchedule {
    MANAGER_SCHEDULE_PARALLEL = 0,
    MANAGER_SCHEDULE_SEQUENTIAL = 1
} ManagerSchedule;

typedef enum FailureStrategy {
    FAILURE_CONTINUE_ON_FAILURE = 0,
    FAILURE_STOP_ON_FAILURE = 1
} FailureStrategy;

typedef struct InstancePlan {
    /** @brief 该计划是否启用；禁用项不会占用运行槽位。 */
    int enabled;
    /** @brief 实例唯一编号，也是端口和输出目录分配依据。 */
    unsigned int instance_id;
    /** @brief 环境程序使用的场景配置路径。 */
    char scenario_path[MANAGER_PATH_SIZE];
    /** @brief 飞控程序使用的飞控配置路径。 */
    char flight_control_path[MANAGER_PATH_SIZE];
    /** @brief 环境程序使用的故障脚本路径。 */
    char faults_path[MANAGER_PATH_SIZE];
    /** @brief 实例显式随机种子。 */
    uint64_t random_seed;
    /** @brief 该实例环境进程 UDP 端口。 */
    unsigned int environment_port;
    /** @brief 该实例飞控进程 UDP 端口。 */
    unsigned int flight_control_port;
} InstancePlan;

typedef struct ManagedInstance {
    /** @brief 本实例的启动计划。 */
    InstancePlan plan;
    /** @brief 飞控子进程号。 */
    pid_t fc_pid;
    /** @brief 环境子进程号。 */
    pid_t env_pid;
    /** @brief 飞控子进程是否已被回收。 */
    int fc_done;
    /** @brief 环境子进程是否已被回收。 */
    int env_done;
    /** @brief waitpid 返回的飞控退出状态。 */
    int fc_status;
    /** @brief waitpid 返回的环境退出状态。 */
    int env_status;
    /** @brief 是否因为前置错误或失败策略而未启动。 */
    int skipped;
    /** @brief 启动前或调度阶段失败原因。 */
    char launch_error[64];
} ManagedInstance;

typedef struct InstanceSummary {
    /** @brief 是否成功读取该实例的 summary.json。 */
    int available;
    /** @brief 实例是否命中目标。 */
    int hit_flag;
    /** @brief 最近点距离，单位米。 */
    double miss_distance_m;
    /** @brief 最近点时刻，单位秒。 */
    double time_of_closest_s;
    /** @brief 实例仿真步数。 */
    unsigned int simulation_steps;
    /** @brief 退出原因字符串。 */
    char exit_reason[64];
    /** @brief 故障进入激活窗口次数。 */
    unsigned int fault_start_count;
    /** @brief 故障离开激活窗口次数。 */
    unsigned int fault_end_count;
    /** @brief 传感器受故障影响的步数。 */
    unsigned int fault_sensor_affected_step_count;
    /** @brief 执行机构受故障影响的步数。 */
    unsigned int fault_actuator_affected_step_count;
} InstanceSummary;

typedef struct ManagerConfig {
    /** @brief 本次任务实际启用的实例数。 */
    unsigned int instance_count;
    /** @brief 同时运行的实例对上限。 */
    unsigned int max_parallel_instances;
    /** @brief 调度模式。 */
    ManagerSchedule schedule;
    /** @brief 失败处理策略。 */
    FailureStrategy failure_strategy;
    /** @brief 全部实例共用的输出根目录。 */
    char output_dir[MANAGER_PATH_SIZE];
    /** @brief 环境 UDP 基础端口。 */
    unsigned int env_base_port;
    /** @brief 飞控 UDP 基础端口。 */
    unsigned int fc_base_port;
    /** @brief 本地 UDP 主机地址字符串。 */
    char host[64];
    /** @brief 缺省随机种子基值，用于 instances[] 未显式配置时派生。 */
    uint64_t base_random_seed;
    /** @brief environment_sim 可执行程序路径，可由 runtime.tools.environment_program 覆盖。 */
    char environment_program[MANAGER_PATH_SIZE];
    /** @brief flight_control_sim 可执行程序路径，可由 runtime.tools.flight_control_program 覆盖。 */
    char flight_control_program[MANAGER_PATH_SIZE];
} ManagerConfig;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *argv0)
{
    (void)printf("usage: %s [--runtime PATH]\n", argv0);
}

/** @brief 将字符串安全写入 JSON 字符串字面量。 */
static void write_json_string(FILE *file, const char *value)
{
    const unsigned char *cursor = (const unsigned char *)(value != 0 ? value : "");

    (void)fputc('"', file);
    while (*cursor != '\0') {
        if (*cursor == '"' || *cursor == '\\') {
            (void)fputc('\\', file);
            (void)fputc((int)*cursor, file);
        } else if (*cursor >= 0x20u) {
            (void)fputc((int)*cursor, file);
        } else {
            (void)fprintf(file, "\\u%04x", (unsigned int)*cursor);
        }
        ++cursor;
    }
    (void)fputc('"', file);
}

/** @brief 解析调度模式字符串。 */
static SimStatus parse_schedule(const char *value, ManagerSchedule *out)
{
    if (value == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strcmp(value, "PARALLEL") == 0) {
        *out = MANAGER_SCHEDULE_PARALLEL;
        return SIM_OK;
    }
    if (strcmp(value, "SEQUENTIAL") == 0) {
        *out = MANAGER_SCHEDULE_SEQUENTIAL;
        return SIM_OK;
    }
    return SIM_ERR_CONFIG;
}

/** @brief 解析失败处理策略字符串。 */
static SimStatus parse_failure_strategy(const char *value, FailureStrategy *out)
{
    if (value == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strcmp(value, "CONTINUE_ON_FAILURE") == 0) {
        *out = FAILURE_CONTINUE_ON_FAILURE;
        return SIM_OK;
    }
    if (strcmp(value, "STOP_ON_FAILURE") == 0) {
        *out = FAILURE_STOP_ON_FAILURE;
        return SIM_OK;
    }
    return SIM_ERR_CONFIG;
}

/** @brief 从运行时配置读取全局编排参数。 */
static SimStatus load_manager_config(const ConfigTree *runtime, ManagerConfig *out)
{
    SimStatus status;
    unsigned int configured_instance_count;
    unsigned int base_seed;
    char text[64];

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    out->schedule = MANAGER_SCHEDULE_PARALLEL;
    out->failure_strategy = FAILURE_CONTINUE_ON_FAILURE;
    (void)snprintf(out->environment_program, sizeof(out->environment_program), "%s", ENVIRONMENT_PROGRAM);
    (void)snprintf(out->flight_control_program, sizeof(out->flight_control_program), "%s", FLIGHT_CONTROL_PROGRAM);

    status = config_get_uint32(runtime, "campaign.instance_count", &configured_instance_count);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "campaign.max_parallel_instances", &out->max_parallel_instances);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "logging.output_dir", out->output_dir, sizeof(out->output_dir));
    if (status != SIM_OK) {
        return status;
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
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "campaign.base_random_seed", &base_seed);
    out->base_random_seed = status == SIM_OK ? (uint64_t)base_seed : UINT64_C(1);

    if (config_get_string(runtime, "campaign.schedule", text, sizeof(text)) == SIM_OK) {
        status = parse_schedule(text, &out->schedule);
        if (status != SIM_OK) {
            return status;
        }
    }
    if (config_get_string(runtime, "campaign.failure_strategy", text, sizeof(text)) == SIM_OK) {
        status = parse_failure_strategy(text, &out->failure_strategy);
        if (status != SIM_OK) {
            return status;
        }
    }
    if (config_get_string(
            runtime,
            "tools.environment_program",
            out->environment_program,
            sizeof(out->environment_program)) != SIM_OK) {
        (void)snprintf(out->environment_program, sizeof(out->environment_program), "%s", ENVIRONMENT_PROGRAM);
    }
    if (config_get_string(
            runtime,
            "tools.flight_control_program",
            out->flight_control_program,
            sizeof(out->flight_control_program)) != SIM_OK) {
        (void)snprintf(out->flight_control_program, sizeof(out->flight_control_program), "%s", FLIGHT_CONTROL_PROGRAM);
    }
    if (configured_instance_count == 0u || configured_instance_count > MAX_INSTANCES) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (out->max_parallel_instances == 0u) {
        out->max_parallel_instances = 1u;
    }
    if (out->schedule == MANAGER_SCHEDULE_SEQUENTIAL) {
        out->max_parallel_instances = 1u;
    }
    return SIM_OK;
}

/** @brief 计算实例端口并校验是否落在 UDP 端口范围内。 */
static SimStatus assign_instance_ports(const ManagerConfig *cfg, InstancePlan *plan)
{
    unsigned int env_port;
    unsigned int fc_port;

    if (cfg == 0 || plan == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    env_port = cfg->env_base_port + (2u * plan->instance_id);
    fc_port = cfg->fc_base_port + (2u * plan->instance_id);
    if (env_port > 65535u || fc_port > 65535u) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    plan->environment_port = env_port;
    plan->flight_control_port = fc_port;
    return SIM_OK;
}

/** @brief 从 runtime.instances[] 读取启用的实例计划。 */
static SimStatus load_instance_plans(
    const ConfigTree *runtime,
    ManagerConfig *cfg,
    InstancePlan *plans)
{
    size_t raw_count = 0u;
    size_t index;
    unsigned int active_count = 0u;
    SimStatus status;

    if (runtime == 0 || cfg == 0 || plans == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_array_count(runtime, "instances", &raw_count);
    if (status != SIM_OK || raw_count == 0u || raw_count > MAX_INSTANCES) {
        return status == SIM_OK ? SIM_ERR_OUT_OF_RANGE : status;
    }

    for (index = 0u; index < raw_count; ++index) {
        char path[96];
        int enabled = 1;
        unsigned int value = 0u;
        InstancePlan plan;

        (void)memset(&plan, 0, sizeof(plan));
        (void)snprintf(path, sizeof(path), "instances[%u].enabled", (unsigned int)index);
        if (config_get_bool(runtime, path, &enabled) != SIM_OK) {
            enabled = 1;
        }
        if (enabled == 0) {
            continue;
        }

        (void)snprintf(path, sizeof(path), "instances[%u].instance_id", (unsigned int)index);
        status = config_get_uint32(runtime, path, &plan.instance_id);
        if (status != SIM_OK) {
            return status;
        }
        (void)snprintf(path, sizeof(path), "instances[%u].scenario", (unsigned int)index);
        status = config_get_string(runtime, path, plan.scenario_path, sizeof(plan.scenario_path));
        if (status != SIM_OK) {
            return status;
        }
        (void)snprintf(path, sizeof(path), "instances[%u].flight_control", (unsigned int)index);
        status = config_get_string(runtime, path, plan.flight_control_path, sizeof(plan.flight_control_path));
        if (status != SIM_OK) {
            return status;
        }
        (void)snprintf(path, sizeof(path), "instances[%u].faults", (unsigned int)index);
        status = config_get_string(runtime, path, plan.faults_path, sizeof(plan.faults_path));
        if (status != SIM_OK) {
            return status;
        }
        (void)snprintf(path, sizeof(path), "instances[%u].random_seed", (unsigned int)index);
        if (config_get_uint32(runtime, path, &value) == SIM_OK) {
            plan.random_seed = (uint64_t)value;
        } else {
            plan.random_seed = cfg->base_random_seed + plan.instance_id;
        }
        status = assign_instance_ports(cfg, &plan);
        if (status != SIM_OK) {
            return status;
        }
        if (active_count >= MAX_INSTANCES) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        plans[active_count++] = plan;
    }

    if (active_count == 0u) {
        return SIM_ERR_CONFIG;
    }
    cfg->instance_count = active_count;
    if (cfg->max_parallel_instances > cfg->instance_count) {
        cfg->max_parallel_instances = cfg->instance_count;
    }
    return SIM_OK;
}

/** @brief 校验实例编号和所有 UDP 端口均唯一。 */
static SimStatus validate_unique_instances(const ManagerConfig *cfg, const InstancePlan *plans)
{
    unsigned int i;
    unsigned int j;

    if (cfg == 0 || plans == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (i = 0u; i < cfg->instance_count; ++i) {
        if (plans[i].environment_port == plans[i].flight_control_port) {
            return SIM_ERR_CONFIG;
        }
        for (j = i + 1u; j < cfg->instance_count; ++j) {
            if (plans[i].instance_id == plans[j].instance_id ||
                plans[i].environment_port == plans[j].environment_port ||
                plans[i].environment_port == plans[j].flight_control_port ||
                plans[i].flight_control_port == plans[j].environment_port ||
                plans[i].flight_control_port == plans[j].flight_control_port) {
                return SIM_ERR_CONFIG;
            }
        }
    }
    return SIM_OK;
}

/** @brief 检查一个 UDP 端口当前是否可绑定。 */
static int udp_port_available(unsigned int port)
{
    int sock;
    struct sockaddr_in addr;
    int ok;

    if (port > 65535u) {
        return 0;
    }
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return 0;
    }
    (void)memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    ok = bind(sock, (const struct sockaddr *)&addr, sizeof(addr)) == 0;
    (void)close(sock);
    return ok;
}

/** @brief 对所有计划端口执行启动前可绑定预检。 */
static SimStatus preflight_ports(const ManagerConfig *cfg, const InstancePlan *plans)
{
    unsigned int i;

    if (cfg == 0 || plans == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (i = 0u; i < cfg->instance_count; ++i) {
        if (udp_port_available(plans[i].environment_port) == 0 ||
            udp_port_available(plans[i].flight_control_port) == 0) {
            (void)fprintf(
                stderr,
                "instance_manager: UDP port unavailable for instance %u env=%u fc=%u\n",
                plans[i].instance_id,
                plans[i].environment_port,
                plans[i].flight_control_port);
            return SIM_ERR_IO;
        }
    }
    return SIM_OK;
}

/** @brief 启动单个子进程。 */
static pid_t launch_process(
    const ManagerConfig *cfg,
    const InstancePlan *plan,
    const char *runtime_path,
    int is_environment)
{
    pid_t pid = fork();
    char instance_arg[32];
    char seed_arg[32];

    if (pid != 0) {
        return pid;
    }

    (void)snprintf(instance_arg, sizeof(instance_arg), "%u", plan->instance_id);
    (void)snprintf(seed_arg, sizeof(seed_arg), "%llu", (unsigned long long)plan->random_seed);
    if (is_environment != 0) {
        execl(
            cfg->environment_program,
            cfg->environment_program,
            "--instance-id",
            instance_arg,
            "--scenario",
            plan->scenario_path,
            "--runtime",
            runtime_path,
            "--faults",
            plan->faults_path,
            "--random-seed",
            seed_arg,
            (char *)0);
    } else {
        execl(
            cfg->flight_control_program,
            cfg->flight_control_program,
            "--instance-id",
            instance_arg,
            "--config",
            plan->flight_control_path,
            "--runtime",
            runtime_path,
            (char *)0);
    }
    _exit(127);
}

/** @brief 等待飞控进程完成 UDP 端口绑定。
 *
 *  管理器先启动飞控，再启动环境。这里通过端口从“可绑定”变为“不可绑定”
 *  判断飞控已经进入接收循环；同时轮询子进程状态，避免飞控配置错误时
 *  仍继续启动环境进程。
 */
static SimStatus wait_for_flight_control_ready(ManagedInstance *instance)
{
    const struct timespec delay = { 0, 10000000L };
    unsigned int attempt;

    if (instance == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (attempt = 0u; attempt < 200u; ++attempt) {
        int child_status = 0;
        pid_t result = waitpid(instance->fc_pid, &child_status, WNOHANG);

        if (result == instance->fc_pid) {
            instance->fc_done = 1;
            instance->fc_status = child_status;
            (void)snprintf(
                instance->launch_error,
                sizeof(instance->launch_error),
                "fc_exited_before_ready");
            return SIM_ERR_IO;
        }
        if (result < 0 && errno != EINTR) {
            (void)snprintf(
                instance->launch_error,
                sizeof(instance->launch_error),
                "fc_wait_failed");
            return SIM_ERR_IO;
        }
        if (udp_port_available(instance->plan.flight_control_port) == 0) {
            return SIM_OK;
        }
        (void)nanosleep(&delay, 0);
    }
    (void)snprintf(
        instance->launch_error,
        sizeof(instance->launch_error),
        "fc_ready_timeout");
    return SIM_ERR_TIMEOUT;
}

/** @brief 启动一个环境和飞控实例对。 */
static SimStatus launch_instance(
    ManagedInstance *instance,
    const char *runtime_path,
    const ManagerConfig *cfg)
{
    SimStatus status;

    if (instance == 0 || runtime_path == 0 || cfg == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    instance->fc_pid = launch_process(cfg, &instance->plan, runtime_path, 0);
    if (instance->fc_pid <= 0) {
        (void)snprintf(instance->launch_error, sizeof(instance->launch_error), "launch_fc_failed");
        return SIM_ERR_INTERNAL;
    }
    status = wait_for_flight_control_ready(instance);
    if (status != SIM_OK) {
        if (instance->fc_done == 0) {
            int child_status = 0;

            (void)kill(instance->fc_pid, SIGTERM);
            (void)waitpid(instance->fc_pid, &child_status, 0);
            instance->fc_done = 1;
            instance->fc_status = child_status;
        }
        return status;
    }
    instance->env_pid = launch_process(cfg, &instance->plan, runtime_path, 1);
    if (instance->env_pid <= 0) {
        (void)snprintf(instance->launch_error, sizeof(instance->launch_error), "launch_env_failed");
        (void)kill(instance->fc_pid, SIGTERM);
        return SIM_ERR_INTERNAL;
    }
    return SIM_OK;
}

/** @brief 按进程号查找实例槽位。 */
static int find_slot_by_pid(ManagedInstance *instances, unsigned int count, pid_t pid)
{
    unsigned int i;

    for (i = 0u; i < count; ++i) {
        if (instances[i].fc_pid == pid || instances[i].env_pid == pid) {
            return (int)i;
        }
    }
    return -1;
}

/** @brief 判断子进程是否以 0 退出。 */
static int process_ok(int status)
{
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

/** @brief 将未启动的实例标记为被失败策略跳过。 */
static void mark_remaining_skipped(
    ManagedInstance *instances,
    unsigned int count,
    unsigned int start_index)
{
    unsigned int i;

    for (i = start_index; i < count; ++i) {
        instances[i].skipped = 1;
        instances[i].fc_done = 1;
        instances[i].env_done = 1;
        instances[i].fc_status = 1;
        instances[i].env_status = 1;
        (void)snprintf(instances[i].launch_error, sizeof(instances[i].launch_error), "skipped_after_failure");
    }
}

/** @brief 读取单实例 summary.json 中用于任务汇总的稳定字段。 */
static void read_instance_summary(
    const ManagerConfig *cfg,
    const InstancePlan *plan,
    InstanceSummary *out)
{
    char path[512];
    ConfigTree tree;
    SimStatus status;

    if (cfg == 0 || plan == 0 || out == 0) {
        return;
    }
    (void)memset(out, 0, sizeof(*out));
    (void)snprintf(out->exit_reason, sizeof(out->exit_reason), "missing_summary");
    (void)snprintf(
        path,
        sizeof(path),
        "%s/instance_%04u/summary.json",
        cfg->output_dir,
        plan->instance_id);
    (void)memset(&tree, 0, sizeof(tree));
    status = config_load_file(path, &tree);
    if (status != SIM_OK) {
        return;
    }
    out->available = 1;
    (void)config_get_bool(&tree, "hit_flag", &out->hit_flag);
    (void)config_get_double(&tree, "miss_distance", &out->miss_distance_m);
    (void)config_get_double(&tree, "time_of_closest_approach", &out->time_of_closest_s);
    (void)config_get_uint32(&tree, "simulation_steps", &out->simulation_steps);
    (void)config_get_string(&tree, "exit_reason", out->exit_reason, sizeof(out->exit_reason));
    (void)config_get_uint32(&tree, "fault_start_count", &out->fault_start_count);
    (void)config_get_uint32(&tree, "fault_end_count", &out->fault_end_count);
    (void)config_get_uint32(
        &tree,
        "fault_sensor_affected_step_count",
        &out->fault_sensor_affected_step_count);
    (void)config_get_uint32(
        &tree,
        "fault_actuator_affected_step_count",
        &out->fault_actuator_affected_step_count);
    config_free(&tree);
}

/** @brief 写出全局任务摘要并返回失败实例数量。 */
static unsigned int write_campaign_summary(
    const ManagerConfig *cfg,
    const ManagedInstance *instances)
{
    char path[512];
    FILE *file;
    unsigned int i;
    unsigned int completed = 0u;
    unsigned int failed = 0u;
    unsigned int summary_available = 0u;
    unsigned int hit_count = 0u;
    unsigned int total_fault_start_count = 0u;
    unsigned int total_fault_end_count = 0u;
    unsigned int total_fault_sensor_affected_steps = 0u;
    unsigned int total_fault_actuator_affected_steps = 0u;
    double min_miss_distance = DBL_MAX;
    InstanceSummary summaries[MAX_INSTANCES];

    (void)mkdir("runs", 0777);
    (void)mkdir(cfg->output_dir, 0777);
    (void)snprintf(path, sizeof(path), "%s/campaign_summary.json", cfg->output_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return cfg->instance_count;
    }

    for (i = 0u; i < cfg->instance_count; ++i) {
        if (process_ok(instances[i].env_status) && process_ok(instances[i].fc_status)) {
            ++completed;
        } else {
            ++failed;
        }
        (void)memset(&summaries[i], 0, sizeof(summaries[i]));
        (void)snprintf(
            summaries[i].exit_reason,
            sizeof(summaries[i].exit_reason),
            instances[i].launch_error[0] != '\0' ? instances[i].launch_error : "process_failed");
        if (process_ok(instances[i].env_status) && process_ok(instances[i].fc_status)) {
            read_instance_summary(cfg, &instances[i].plan, &summaries[i]);
        }
        if (summaries[i].available != 0) {
            ++summary_available;
            if (summaries[i].hit_flag != 0) {
                ++hit_count;
            }
            if (summaries[i].miss_distance_m < min_miss_distance) {
                min_miss_distance = summaries[i].miss_distance_m;
            }
            total_fault_start_count += summaries[i].fault_start_count;
            total_fault_end_count += summaries[i].fault_end_count;
            total_fault_sensor_affected_steps += summaries[i].fault_sensor_affected_step_count;
            total_fault_actuator_affected_steps += summaries[i].fault_actuator_affected_step_count;
        }
    }

    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"instance_count\": %u,\n", cfg->instance_count);
    (void)fprintf(file, "  \"schedule\": ");
    write_json_string(
        file,
        cfg->schedule == MANAGER_SCHEDULE_SEQUENTIAL ? "SEQUENTIAL" : "PARALLEL");
    (void)fprintf(file, ",\n");
    (void)fprintf(file, "  \"completed_count\": %u,\n", completed);
    (void)fprintf(file, "  \"failed_count\": %u,\n", failed);
    (void)fprintf(file, "  \"summary_available_count\": %u,\n", summary_available);
    (void)fprintf(file, "  \"hit_count\": %u,\n", hit_count);
    (void)fprintf(
        file,
        "  \"min_miss_distance\": %.6f,\n",
        summary_available > 0u ? min_miss_distance : 0.0);
    (void)fprintf(file, "  \"total_fault_start_count\": %u,\n", total_fault_start_count);
    (void)fprintf(file, "  \"total_fault_end_count\": %u,\n", total_fault_end_count);
    (void)fprintf(
        file,
        "  \"total_fault_sensor_affected_step_count\": %u,\n",
        total_fault_sensor_affected_steps);
    (void)fprintf(
        file,
        "  \"total_fault_actuator_affected_step_count\": %u,\n",
        total_fault_actuator_affected_steps);
    (void)fprintf(file, "  \"instances\": [\n");
    for (i = 0u; i < cfg->instance_count; ++i) {
        (void)fprintf(file, "    { \"instance_id\": %u, ", instances[i].plan.instance_id);
        (void)fprintf(
            file,
            "\"environment_port\": %u, \"flight_control_port\": %u, ",
            instances[i].plan.environment_port,
            instances[i].plan.flight_control_port);
        (void)fprintf(
            file,
            "\"random_seed\": %llu, ",
            (unsigned long long)instances[i].plan.random_seed);
        (void)fprintf(file, "\"scenario\": ");
        write_json_string(file, instances[i].plan.scenario_path);
        (void)fprintf(file, ", \"flight_control\": ");
        write_json_string(file, instances[i].plan.flight_control_path);
        (void)fprintf(file, ", \"faults\": ");
        write_json_string(file, instances[i].plan.faults_path);
        (void)fprintf(
            file,
            ", \"env_status\": %d, \"fc_status\": %d, "
            "\"summary_available\": %s, \"hit_flag\": %s, "
            "\"miss_distance\": %.6f, \"exit_reason\": ",
            process_ok(instances[i].env_status) ? 0 : 1,
            process_ok(instances[i].fc_status) ? 0 : 1,
            summaries[i].available != 0 ? "true" : "false",
            summaries[i].hit_flag != 0 ? "true" : "false",
            summaries[i].miss_distance_m);
        write_json_string(file, summaries[i].exit_reason);
        (void)fprintf(
            file,
            ", \"fault_start_count\": %u, \"fault_end_count\": %u, "
            "\"fault_sensor_affected_step_count\": %u, "
            "\"fault_actuator_affected_step_count\": %u }%s\n",
            summaries[i].fault_start_count,
            summaries[i].fault_end_count,
            summaries[i].fault_sensor_affected_step_count,
            summaries[i].fault_actuator_affected_step_count,
            i + 1u == cfg->instance_count ? "" : ",");
    }
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    (void)fclose(file);
    return failed;
}

/** @brief 将实例计划复制到运行槽位。 */
static void initialize_managed_instances(
    ManagedInstance *instances,
    const InstancePlan *plans,
    unsigned int count)
{
    unsigned int i;

    for (i = 0u; i < count; ++i) {
        instances[i].plan = plans[i];
        instances[i].fc_pid = -1;
        instances[i].env_pid = -1;
        instances[i].fc_status = 1;
        instances[i].env_status = 1;
    }
}

/** @brief 解析运行配置，调度实例对并生成任务摘要。 */
int main(int argc, char **argv)
{
    const char *runtime_path = DEFAULT_RUNTIME_CONFIG;
    ConfigTree runtime;
    ManagerConfig cfg;
    InstancePlan plans[MAX_INSTANCES];
    ManagedInstance instances[MAX_INSTANCES];
    unsigned int next_to_launch = 0u;
    unsigned int running = 0u;
    unsigned int completed = 0u;
    unsigned int failed_count;
    int stop_launching = 0;
    SimStatus status;
    int i;

    (void)memset(&runtime, 0, sizeof(runtime));
    (void)memset(&cfg, 0, sizeof(cfg));
    (void)memset(plans, 0, sizeof(plans));
    (void)memset(instances, 0, sizeof(instances));

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--runtime") == 0 && (i + 1) < argc) {
            runtime_path = argv[++i];
            continue;
        }
        print_usage(argv[0]);
        return 2;
    }

    status = config_load_file(runtime_path, &runtime);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "instance_manager: failed to load runtime config: %s\n",
            sim_status_to_string(status));
        return 1;
    }
    status = config_validate_schema(&runtime, 1u);
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "campaign");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "network");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "logging");
    }
    if (status == SIM_OK) {
        status = load_manager_config(&runtime, &cfg);
    }
    if (status == SIM_OK) {
        status = load_instance_plans(&runtime, &cfg, plans);
    }
    config_free(&runtime);
    if (status == SIM_OK) {
        status = validate_unique_instances(&cfg, plans);
    }
    if (status == SIM_OK) {
        status = preflight_ports(&cfg, plans);
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "instance_manager: invalid runtime config: %s\n",
            sim_status_to_string(status));
        return 1;
    }
    initialize_managed_instances(instances, plans, cfg.instance_count);

    while (completed < cfg.instance_count) {
        while (stop_launching == 0 &&
            next_to_launch < cfg.instance_count &&
            running < cfg.max_parallel_instances) {
            ManagedInstance *slot = &instances[next_to_launch];

            status = launch_instance(slot, runtime_path, &cfg);
            if (status != SIM_OK) {
                slot->env_status = 1;
                slot->fc_status = 1;
                slot->env_done = 1;
                slot->fc_done = 1;
                ++completed;
                if (cfg.failure_strategy == FAILURE_STOP_ON_FAILURE) {
                    stop_launching = 1;
                }
            } else {
                ++running;
                (void)printf(
                    "launched instance %u env_port=%u fc_port=%u seed=%llu\n",
                    slot->plan.instance_id,
                    slot->plan.environment_port,
                    slot->plan.flight_control_port,
                    (unsigned long long)slot->plan.random_seed);
            }
            ++next_to_launch;
        }

        if (running == 0u) {
            if (stop_launching != 0 && next_to_launch < cfg.instance_count) {
                mark_remaining_skipped(instances, cfg.instance_count, next_to_launch);
                completed = cfg.instance_count;
            } else if (next_to_launch >= cfg.instance_count) {
                break;
            }
            continue;
        }

        {
            int child_status = 0;
            pid_t pid = wait(&child_status);
            int slot_index;

            if (pid < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            slot_index = find_slot_by_pid(instances, cfg.instance_count, pid);
            if (slot_index >= 0) {
                ManagedInstance *slot = &instances[slot_index];

                if (slot->env_pid == pid) {
                    slot->env_done = 1;
                    slot->env_status = child_status;
                }
                if (slot->fc_pid == pid) {
                    slot->fc_done = 1;
                    slot->fc_status = child_status;
                }
                if (slot->env_done != 0 && slot->fc_done != 0) {
                    --running;
                    ++completed;
                    (void)printf("completed instance %u env_ok=%d fc_ok=%d\n",
                        slot->plan.instance_id,
                        process_ok(slot->env_status),
                        process_ok(slot->fc_status));
                    if ((process_ok(slot->env_status) == 0 || process_ok(slot->fc_status) == 0) &&
                        cfg.failure_strategy == FAILURE_STOP_ON_FAILURE) {
                        stop_launching = 1;
                    }
                }
            }
        }
    }

    failed_count = write_campaign_summary(&cfg, instances);
    (void)printf(
        "instance_manager completed %u instances failed=%u\n",
        completed,
        failed_count);
    return failed_count == 0u ? 0 : 1;
}
