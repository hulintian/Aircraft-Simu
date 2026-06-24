/** @file main.c
 *  @brief 多实例编排管理器实现。
 *
 *  该工具根据 runtime.json 启动成对的环境与飞控进程，并收集运行结果。
 */
#include "common/config.h"
#include "common/status.h"

#include <errno.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define DEFAULT_RUNTIME_CONFIG "configs/baseline/runtime.json"
#define DEFAULT_SCENARIO_CONFIG "configs/baseline/scenario.json"
#define DEFAULT_FC_CONFIG "configs/baseline/flight_control.json"
#define DEFAULT_FAULTS_CONFIG "configs/baseline/faults.json"
#define MAX_INSTANCES 128u

typedef struct ManagedInstance {
    /** @brief 实例唯一编号，也是端口和输出目录分配依据。 */
    unsigned int instance_id;
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
    /** @brief 本次任务计划运行的实例数。 */
    unsigned int instance_count;
    /** @brief 同时运行的实例对上限。 */
    unsigned int max_parallel_instances;
    /** @brief 全部实例共用的输出根目录。 */
    char output_dir[256];
} ManagerConfig;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *argv0)
{
    (void)printf("usage: %s [--runtime PATH]\n", argv0);
}

/** @brief 从运行时配置读取实例编排参数。 */
static SimStatus load_manager_config(const ConfigTree *runtime, ManagerConfig *out)
{
    SimStatus status;

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_uint32(runtime, "campaign.instance_count", &out->instance_count);
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
    if (out->instance_count == 0u || out->instance_count > MAX_INSTANCES) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (out->max_parallel_instances == 0u) {
        out->max_parallel_instances = 1u;
    }
    if (out->max_parallel_instances > out->instance_count) {
        out->max_parallel_instances = out->instance_count;
    }
    return SIM_OK;
}

/** @brief 启动单个子进程。 */
static pid_t launch_process(
    const char *program,
    unsigned int instance_id,
    const char *runtime_path,
    int is_environment)
{
    pid_t pid = fork();
    char instance_arg[32];

    if (pid != 0) {
        return pid;
    }

    (void)snprintf(instance_arg, sizeof(instance_arg), "%u", instance_id);
    if (is_environment != 0) {
        execl(
            program,
            program,
            "--instance-id",
            instance_arg,
            "--scenario",
            DEFAULT_SCENARIO_CONFIG,
            "--runtime",
            runtime_path,
            "--faults",
            DEFAULT_FAULTS_CONFIG,
            (char *)0);
    } else {
        execl(
            program,
            program,
            "--instance-id",
            instance_arg,
            "--config",
            DEFAULT_FC_CONFIG,
            "--runtime",
            runtime_path,
            (char *)0);
    }
    _exit(127);
}

/** @brief 启动一个环境和飞控实例对。 */
static SimStatus launch_instance(ManagedInstance *instance, const char *runtime_path)
{
    if (instance == 0 || runtime_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    instance->fc_pid = launch_process("./build/flight_control_sim/flight_control_sim", instance->instance_id, runtime_path, 0);
    if (instance->fc_pid <= 0) {
        return SIM_ERR_INTERNAL;
    }
    (void)sleep(1u);
    instance->env_pid = launch_process("./build/environment_sim/environment_sim", instance->instance_id, runtime_path, 1);
    if (instance->env_pid <= 0) {
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

/** @brief 读取单实例 summary.json 中用于任务汇总的稳定字段。 */
static void read_instance_summary(
    const ManagerConfig *cfg,
    unsigned int instance_id,
    InstanceSummary *out)
{
    char path[512];
    ConfigTree tree;
    SimStatus status;

    if (cfg == 0 || out == 0) {
        return;
    }
    (void)memset(out, 0, sizeof(*out));
    (void)snprintf(out->exit_reason, sizeof(out->exit_reason), "missing_summary");
    (void)snprintf(
        path,
        sizeof(path),
        "%s/instance_%04u/summary.json",
        cfg->output_dir,
        instance_id);
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

/** @brief 写出全局任务摘要。 */
static void write_campaign_summary(const ManagerConfig *cfg, const ManagedInstance *instances)
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
        return;
    }

    for (i = 0u; i < cfg->instance_count; ++i) {
        if (process_ok(instances[i].env_status) && process_ok(instances[i].fc_status)) {
            ++completed;
        } else {
            ++failed;
        }
        (void)memset(&summaries[i], 0, sizeof(summaries[i]));
        (void)snprintf(summaries[i].exit_reason, sizeof(summaries[i].exit_reason), "process_failed");
        if (process_ok(instances[i].env_status) && process_ok(instances[i].fc_status)) {
            read_instance_summary(cfg, instances[i].instance_id, &summaries[i]);
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
        (void)fprintf(file,
            "    { \"instance_id\": %u, \"env_status\": %d, \"fc_status\": %d, "
            "\"summary_available\": %s, \"hit_flag\": %s, "
            "\"miss_distance\": %.6f, \"exit_reason\": \"%s\", "
            "\"fault_start_count\": %u, \"fault_end_count\": %u, "
            "\"fault_sensor_affected_step_count\": %u, "
            "\"fault_actuator_affected_step_count\": %u }%s\n",
            instances[i].instance_id,
            process_ok(instances[i].env_status) ? 0 : 1,
            process_ok(instances[i].fc_status) ? 0 : 1,
            summaries[i].available != 0 ? "true" : "false",
            summaries[i].hit_flag != 0 ? "true" : "false",
            summaries[i].miss_distance_m,
            summaries[i].exit_reason,
            summaries[i].fault_start_count,
            summaries[i].fault_end_count,
            summaries[i].fault_sensor_affected_step_count,
            summaries[i].fault_actuator_affected_step_count,
            i + 1u == cfg->instance_count ? "" : ",");
    }
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    (void)fclose(file);
}

/** @brief 管理器入口。 */
/** @brief 解析运行配置，调度实例对并生成任务摘要。 */
int main(int argc, char **argv)
{
    const char *runtime_path = DEFAULT_RUNTIME_CONFIG;
    ConfigTree runtime;
    ManagerConfig cfg;
    ManagedInstance instances[MAX_INSTANCES];
    unsigned int next_to_launch = 0u;
    unsigned int running = 0u;
    unsigned int completed = 0u;
    SimStatus status;
    int i;

    memset(&runtime, 0, sizeof(runtime));
    memset(&cfg, 0, sizeof(cfg));
    memset(instances, 0, sizeof(instances));

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
    status = load_manager_config(&runtime, &cfg);
    if (status == SIM_OK) {
        status = config_validate_schema(&runtime, 1u);
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "campaign");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "network");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime, "logging");
    }
    config_free(&runtime);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "instance_manager: invalid runtime config: %s\n",
            sim_status_to_string(status));
        return 1;
    }

    while (completed < cfg.instance_count) {
        while (next_to_launch < cfg.instance_count && running < cfg.max_parallel_instances) {
            ManagedInstance *slot = &instances[next_to_launch];
            slot->instance_id = next_to_launch;
            status = launch_instance(slot, runtime_path);
            if (status != SIM_OK) {
                slot->env_status = 1;
                slot->fc_status = 1;
                slot->env_done = 1;
                slot->fc_done = 1;
                ++completed;
            } else {
                ++running;
                (void)printf("launched instance %u\n", slot->instance_id);
            }
            ++next_to_launch;
        }

        if (running > 0u) {
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
                        slot->instance_id,
                        process_ok(slot->env_status),
                        process_ok(slot->fc_status));
                }
            }
        }
    }

    write_campaign_summary(&cfg, instances);
    (void)printf("instance_manager completed %u instances\n", completed);
    return 0;
}
