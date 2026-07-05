/** @file main.c
 *  @brief 使用 sensor_log.bin 重新驱动飞控静态库生成 command_log.bin。
 */
#include "common/config.h"
#include "common/packet.h"
#include "common/status.h"
#include "fc/fc_state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct ReplayOptions {
    const char *config_path;
    const char *input_path;
    const char *output_path;
    uint32_t instance_id;
} ReplayOptions;

static void print_usage(const char *argv0)
{
    (void)fprintf(
        stderr,
        "usage: %s --instance-id N --config PATH --input sensor_log.bin --output command_log.bin\n",
        argv0);
}

static SimStatus parse_args(int argc, char **argv, ReplayOptions *out)
{
    int i;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    out->config_path = "configs/baseline/flight_control.json";
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[i], "--instance-id") == 0 && (i + 1) < argc) {
            out->instance_id = (uint32_t)strtoul(argv[++i], 0, 10);
            continue;
        }
        if (strcmp(argv[i], "--config") == 0 && (i + 1) < argc) {
            out->config_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--input") == 0 && (i + 1) < argc) {
            out->input_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--output") == 0 && (i + 1) < argc) {
            out->output_path = argv[++i];
            continue;
        }
        return SIM_ERR_CONFIG;
    }
    return out->input_path != 0 && out->output_path != 0 ? SIM_OK : SIM_ERR_CONFIG;
}

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

static SimStatus load_guidance_config(const ConfigTree *config, GuidancePngConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "guidance.navigation_constant", &out->navigation_constant);
    if (status == SIM_OK) {
        status = config_get_double(config, "guidance.max_accel_mps2", &out->max_accel_mps2);
    }
    if (status == SIM_OK) {
        status = config_get_double(config, "guidance.max_accel_rate_mps3", &out->max_accel_rate_mps3);
    }
    return status;
}

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

static SimStatus load_safety_config(const ConfigTree *config, FcSafetyConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "safety.sensor_timeout_s", &out->sensor_timeout_s);
    if (status == SIM_OK) {
        status = config_get_double(config, "safety.command_hold_s", &out->command_hold_s);
    }
    if (status == SIM_OK) {
        status = config_get_bool(config, "safety.reject_nan", &out->reject_nan);
    }
    if (status == SIM_OK) {
        status = config_get_bool(config, "safety.reject_old_seq", &out->reject_old_seq);
    }
    out->max_consecutive_bad_frames = 3u;
    return status;
}

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

static SimStatus load_controller_config(const char *path, FlightControllerConfig *out)
{
    ConfigTree tree;
    SimStatus status;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(&tree, 0, sizeof(tree));
    (void)memset(out, 0, sizeof(*out));
    status = config_load_file(path, &tree);
    if (status == SIM_OK) {
        status = config_validate_schema(&tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_require_section(&tree, "guidance");
    }
    if (status == SIM_OK) {
        status = config_require_section(&tree, "safety");
    }
    if (status == SIM_OK) {
        status = load_guidance_config(&tree, &out->guidance);
    }
    if (status == SIM_OK) {
        status = load_autopilot_config(&tree, &out->autopilot);
    }
    if (status == SIM_OK) {
        status = load_safety_config(&tree, &out->safety);
    }
    if (status == SIM_OK) {
        status = load_scheduler_config(&tree, out);
    }
    config_free(&tree);
    return status;
}

static SimStatus replay_sensor_log(const ReplayOptions *options)
{
    FlightControllerConfig config;
    FlightController controller;
    FILE *input;
    FILE *output;
    unsigned char sensor_packet[SIM_SENSOR_PACKET_WIRE_SIZE];
    unsigned char command_packet[SIM_CONTROL_PACKET_WIRE_SIZE];
    SimStatus status;

    if (options == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = load_controller_config(options->config_path, &config);
    if (status != SIM_OK) {
        return status;
    }
    status = flight_controller_init(&controller, &config);
    if (status != SIM_OK) {
        return status;
    }
    input = fopen(options->input_path, "rb");
    if (input == 0) {
        return SIM_ERR_IO;
    }
    output = fopen(options->output_path, "wb");
    if (output == 0) {
        (void)fclose(input);
        return SIM_ERR_IO;
    }
    for (;;) {
        const size_t got = fread(sensor_packet, 1u, sizeof(sensor_packet), input);
        SensorFrame sensor;
        ControlCommand command;
        size_t command_size = 0u;

        if (got == 0u) {
            status = ferror(input) == 0 ? SIM_OK : SIM_ERR_IO;
            break;
        }
        if (got != sizeof(sensor_packet)) {
            status = SIM_ERR_BAD_PACKET;
            break;
        }
        status = packet_decode_sensor_frame(
            sensor_packet,
            sizeof(sensor_packet),
            options->instance_id,
            &sensor);
        if (status != SIM_OK) {
            break;
        }
        status = flight_controller_step(&controller, &sensor, &command);
        if (status != SIM_OK) {
            break;
        }
        status = packet_encode_control_command(
            options->instance_id,
            &command,
            command_packet,
            sizeof(command_packet),
            &command_size);
        if (status != SIM_OK) {
            break;
        }
        if (fwrite(command_packet, 1u, command_size, output) != command_size) {
            status = SIM_ERR_IO;
            break;
        }
    }
    (void)fclose(input);
    if (fclose(output) != 0 && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    return status;
}

int main(int argc, char **argv)
{
    ReplayOptions options;
    SimStatus status = parse_args(argc, argv, &options);

    if (status != SIM_OK) {
        print_usage(argv[0]);
        return 2;
    }
    status = replay_sensor_log(&options);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "replay: failed: %s\n", sim_status_to_string(status));
        return 1;
    }
    return 0;
}
