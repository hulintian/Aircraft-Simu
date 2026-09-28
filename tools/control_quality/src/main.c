/** @file main.c
 *  @brief 使用真实飞控主链路执行标准机动并输出控制品质报告。
 */
#include "common/config.h"
#include "common/protocol.h"
#include "common/status.h"
#include "fc/control_quality.h"
#include "fc/fc_health.h"
#include "fc/fc_state.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUALITY_MAX_SAMPLES 10000u

typedef struct QualityOptions {
    const char *flight_control_path;
    const char *criteria_path;
    const char *output_path;
} QualityOptions;

typedef struct QualityCriteria {
    double dt_s;
    double duration_s;
    double target_accel_mps2;
    double closing_velocity_mps;
    double step_time_s;
    double reversal_time_s;
    double dropout_start_s;
    double dropout_duration_s;
    double settling_band_fraction;
    double max_rise_time_s;
    double max_step_settling_time_s;
    double max_reversal_settling_time_s;
    double max_recovery_settling_time_s;
    double max_overshoot_percent;
    double max_steady_state_error;
    double max_saturation_fraction;
    double rate_tolerance_fraction;
} QualityCriteria;

typedef enum ManeuverKind {
    MANEUVER_STEP = 0,
    MANEUVER_REVERSAL = 1,
    MANEUVER_RECOVERY = 2
} ManeuverKind;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *program)
{
    (void)fprintf(
        stderr,
        "usage: %s --flight-control PATH --criteria PATH [--output PATH]\n",
        program);
}

/** @brief 解析命令行参数。 */
static SimStatus parse_args(int argc, char **argv, QualityOptions *out)
{
    int index;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[index], "--flight-control") == 0 && index + 1 < argc) {
            out->flight_control_path = argv[++index];
        } else if (strcmp(argv[index], "--criteria") == 0 && index + 1 < argc) {
            out->criteria_path = argv[++index];
        } else if (strcmp(argv[index], "--output") == 0 && index + 1 < argc) {
            out->output_path = argv[++index];
        } else {
            return SIM_ERR_CONFIG;
        }
    }
    return out->flight_control_path != 0 && out->criteria_path != 0 ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 读取飞控配置中标准机动所需的控制器参数。 */
static SimStatus load_flight_controller_config(const char *path, FlightControllerConfig *out)
{
    ConfigTree tree;
    SimStatus status;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    status = config_load_file(path, &tree);
    if (status != SIM_OK) {
        return status;
    }
    status = config_validate_json(&tree);
    if (status == SIM_OK) {
        status = config_validate_schema(&tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "scheduler.base_rate_hz", &out->scheduler_base_rate_hz);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "guidance.navigation_constant", &out->guidance.navigation_constant);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "guidance.max_accel_mps2", &out->guidance.max_accel_mps2);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "guidance.max_accel_rate_mps3",
            &out->guidance.max_accel_rate_mps3);
    }
    if (status == SIM_OK) {
        status = config_get_bool(
            &tree,
            "autopilot.enable_attitude_loop",
            &out->autopilot.enable_attitude_loop);
    }
    if (status == SIM_OK) {
        status = config_get_bool(
            &tree,
            "autopilot.enable_control_allocation",
            &out->autopilot.enable_control_allocation);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.max_attitude_cmd_rad",
            &out->autopilot.max_attitude_cmd_rad);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.max_body_rate_cmd_radps",
            &out->autopilot.max_body_rate_cmd_radps);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.attitude_time_constant_s",
            &out->autopilot.attitude_time_constant_s);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.gyro_damping_gain",
            &out->autopilot.gyro_damping_gain);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.fin_accel_effectiveness_mps2_per_rad",
            &out->autopilot.fin_accel_effectiveness_mps2_per_rad);
    }
    if (status == SIM_OK) {
        status = config_get_double(
            &tree,
            "autopilot.max_fin_deflection_rad",
            &out->autopilot.max_fin_deflection_rad);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "safety.sensor_timeout_s", &out->safety.sensor_timeout_s);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "safety.command_hold_s", &out->safety.command_hold_s);
    }
    if (status == SIM_OK) {
        status = config_get_bool(&tree, "safety.reject_nan", &out->safety.reject_nan);
    }
    if (status == SIM_OK) {
        status = config_get_bool(&tree, "safety.reject_old_seq", &out->safety.reject_old_seq);
    }
    out->safety.max_consecutive_bad_frames = 3u;
    config_free(&tree);
    return status;
}

/** @brief 从版本化 JSON 读取标准机动与验收阈值。 */
static SimStatus load_quality_criteria(const char *path, QualityCriteria *out)
{
    ConfigTree tree;
    SimStatus status;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    status = config_load_file(path, &tree);
    if (status != SIM_OK) {
        return status;
    }
    status = config_validate_json(&tree);
    if (status == SIM_OK) {
        status = config_validate_schema(&tree, 1u);
    }
#define READ_CRITERION(json_path, field) \
    do { \
        if (status == SIM_OK) { \
            status = config_get_double(&tree, json_path, &out->field); \
        } \
    } while (0)
    READ_CRITERION("maneuvers.dt_s", dt_s);
    READ_CRITERION("maneuvers.duration_s", duration_s);
    READ_CRITERION("maneuvers.target_accel_mps2", target_accel_mps2);
    READ_CRITERION("maneuvers.closing_velocity_mps", closing_velocity_mps);
    READ_CRITERION("maneuvers.step_time_s", step_time_s);
    READ_CRITERION("maneuvers.reversal_time_s", reversal_time_s);
    READ_CRITERION("maneuvers.dropout_start_s", dropout_start_s);
    READ_CRITERION("maneuvers.dropout_duration_s", dropout_duration_s);
    READ_CRITERION("criteria.settling_band_fraction", settling_band_fraction);
    READ_CRITERION("criteria.max_rise_time_s", max_rise_time_s);
    READ_CRITERION("criteria.max_step_settling_time_s", max_step_settling_time_s);
    READ_CRITERION("criteria.max_reversal_settling_time_s", max_reversal_settling_time_s);
    READ_CRITERION("criteria.max_recovery_settling_time_s", max_recovery_settling_time_s);
    READ_CRITERION("criteria.max_overshoot_percent", max_overshoot_percent);
    READ_CRITERION("criteria.max_steady_state_error", max_steady_state_error);
    READ_CRITERION("criteria.max_saturation_fraction", max_saturation_fraction);
    READ_CRITERION("criteria.rate_tolerance_fraction", rate_tolerance_fraction);
#undef READ_CRITERION
    config_free(&tree);
    if (status != SIM_OK) {
        return status;
    }
    if (out->dt_s <= 0.0 || out->duration_s <= out->dt_s ||
        out->target_accel_mps2 <= 0.0 || out->closing_velocity_mps <= 0.0 ||
        out->step_time_s < 0.0 || out->reversal_time_s < 0.0 ||
        out->dropout_start_s < 0.0 || out->dropout_duration_s <= 0.0 ||
        out->settling_band_fraction <= 0.0 || out->rate_tolerance_fraction < 0.0) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    return SIM_OK;
}

/** @brief 构造标准机动的一帧确定性传感器输入。 */
static SensorFrame make_sensor(
    uint32_t seq,
    double time_s,
    double dt_s,
    double closing_velocity_mps,
    double los_rate_z,
    int seeker_valid)
{
    SensorFrame sensor;

    (void)memset(&sensor, 0, sizeof(sensor));
    sensor.seq = seq;
    sensor.sim_time = time_s;
    sensor.dt = dt_s;
    sensor.missile_vel_ecef_meas = vec3_make(300.0, 0.0, 0.0);
    sensor.missile_lat_rad_meas = 0.0;
    sensor.missile_lon_rad_meas = 0.0;
    sensor.missile_height_m_meas = 1000.0;
    sensor.missile_height_agl_m_meas = 1000.0;
    sensor.target_range_meas = 10000.0;
    sensor.target_los_unit_ecef_meas = vec3_make(1.0, 0.0, 0.0);
    sensor.target_los_rate_ecef_meas = vec3_make(0.0, 0.0, los_rate_z);
    sensor.target_closing_velocity_meas = closing_velocity_mps;
    sensor.sensor_valid_flags =
        SIM_SENSOR_VALID_IMU_GYRO |
        SIM_SENSOR_VALID_ACCEL |
        SIM_SENSOR_VALID_SPEED |
        SIM_SENSOR_VALID_GEODETIC;
    if (seeker_valid != 0) {
        sensor.sensor_valid_flags |= SIM_SENSOR_VALID_SEEKER;
    }
    return sensor;
}

/** @brief 运行一个标准工况并提取目标轴响应。 */
static SimStatus run_maneuver(
    ManeuverKind kind,
    const FlightControllerConfig *controller_config,
    const QualityCriteria *criteria,
    ControlQualitySample *samples,
    size_t sample_count,
    double *event_time_s,
    double *target_out)
{
    FlightController controller;
    size_t index;
    const double los_rate = criteria->target_accel_mps2 /
        (controller_config->guidance.navigation_constant * criteria->closing_velocity_mps);
    SimStatus status;

    status = flight_controller_init(&controller, controller_config);
    if (status != SIM_OK) {
        return status;
    }
    for (index = 0u; index < sample_count; ++index) {
        const double time_s = criteria->dt_s * (double)(index + 1u);
        double signed_los_rate = los_rate;
        int seeker_valid = 1;
        SensorFrame sensor;
        ControlCommand command;

        if (kind == MANEUVER_STEP) {
            signed_los_rate = time_s >= criteria->step_time_s ? los_rate : 0.0;
            *event_time_s = criteria->step_time_s;
            *target_out = criteria->target_accel_mps2;
        } else if (kind == MANEUVER_REVERSAL) {
            signed_los_rate = time_s >= criteria->reversal_time_s ? -los_rate : los_rate;
            *event_time_s = criteria->reversal_time_s;
            *target_out = -criteria->target_accel_mps2;
        } else {
            const double dropout_end = criteria->dropout_start_s + criteria->dropout_duration_s;

            seeker_valid = !(time_s >= criteria->dropout_start_s && time_s < dropout_end);
            *event_time_s = dropout_end;
            *target_out = criteria->target_accel_mps2;
        }
        sensor = make_sensor(
            (uint32_t)(index + 1u),
            time_s,
            criteria->dt_s,
            criteria->closing_velocity_mps,
            signed_los_rate,
            seeker_valid);
        status = flight_controller_step(&controller, &sensor, &command);
        if (status != SIM_OK || !vec3_isfinite(command.accel_cmd_ecef)) {
            return status == SIM_OK ? SIM_ERR_NUMERIC : status;
        }
        samples[index].time_s = time_s;
        samples[index].response = command.accel_cmd_ecef.y;
        samples[index].saturated =
            (command.command_status &
                (FC_HEALTH_WARNING_RATE_LIMITED | FC_HEALTH_WARNING_COMMAND_LIMITED)) != 0u;
    }
    return SIM_OK;
}

/** @brief 判断一组指标是否满足版本化阈值。 */
static int metrics_pass(
    const ControlQualityMetrics *metrics,
    const QualityCriteria *criteria,
    double max_settling_time_s,
    double max_command_rate)
{
    return metrics->rise_time_s <= criteria->max_rise_time_s &&
        metrics->settling_time_s <= max_settling_time_s &&
        metrics->overshoot_percent <= criteria->max_overshoot_percent &&
        fabs(metrics->steady_state_error) <= criteria->max_steady_state_error &&
        metrics->max_abs_rate <= max_command_rate * (1.0 + criteria->rate_tolerance_fraction) &&
        metrics->saturation_fraction <= criteria->max_saturation_fraction;
}

/** @brief 写出单个工况的稳定 JSON 对象。 */
static void write_metrics(
    FILE *file,
    const char *name,
    const ControlQualityMetrics *metrics,
    int pass,
    int trailing_comma)
{
    (void)fprintf(file, "    \"%s\": {\n", name);
    (void)fprintf(file, "      \"pass\": %s,\n", pass != 0 ? "true" : "false");
    (void)fprintf(file, "      \"rise_time_s\": %.9f,\n", metrics->rise_time_s);
    (void)fprintf(file, "      \"settling_time_s\": %.9f,\n", metrics->settling_time_s);
    (void)fprintf(file, "      \"overshoot_percent\": %.9f,\n", metrics->overshoot_percent);
    (void)fprintf(file, "      \"steady_state_error\": %.9f,\n", metrics->steady_state_error);
    (void)fprintf(file, "      \"max_abs_rate\": %.9f,\n", metrics->max_abs_rate);
    (void)fprintf(file, "      \"saturation_fraction\": %.9f,\n", metrics->saturation_fraction);
    (void)fprintf(file, "      \"sample_count\": %zu\n", metrics->evaluated_sample_count);
    (void)fprintf(file, "    }%s\n", trailing_comma != 0 ? "," : "");
}

int main(int argc, char **argv)
{
    QualityOptions options;
    FlightControllerConfig controller_config;
    QualityCriteria criteria;
    ControlQualitySample samples[QUALITY_MAX_SAMPLES];
    ControlQualityMetrics step_metrics;
    ControlQualityMetrics reversal_metrics;
    ControlQualityMetrics recovery_metrics;
    size_t sample_count;
    double event_time_s;
    double target;
    int step_pass;
    int reversal_pass;
    int recovery_pass;
    FILE *output = stdout;
    SimStatus status;

    status = parse_args(argc, argv, &options);
    if (status == SIM_OK) {
        status = load_flight_controller_config(options.flight_control_path, &controller_config);
    }
    if (status == SIM_OK) {
        status = load_quality_criteria(options.criteria_path, &criteria);
    }
    if (status != SIM_OK) {
        print_usage(argv[0]);
        (void)fprintf(stderr, "control_quality_report: configuration failed: %s\n", sim_status_to_string(status));
        return 2;
    }
    sample_count = (size_t)ceil(criteria.duration_s / criteria.dt_s);
    if (sample_count < 2u || sample_count > QUALITY_MAX_SAMPLES ||
        criteria.target_accel_mps2 > controller_config.guidance.max_accel_mps2) {
        (void)fprintf(stderr, "control_quality_report: maneuver is outside configured limits\n");
        return 2;
    }
    status = run_maneuver(
        MANEUVER_STEP,
        &controller_config,
        &criteria,
        samples,
        sample_count,
        &event_time_s,
        &target);
    if (status == SIM_OK) {
        status = control_quality_analyze_step(
            samples,
            sample_count,
            event_time_s,
            target,
            criteria.settling_band_fraction,
            &step_metrics);
    }
    if (status == SIM_OK) {
        status = run_maneuver(
            MANEUVER_REVERSAL,
            &controller_config,
            &criteria,
            samples,
            sample_count,
            &event_time_s,
            &target);
    }
    if (status == SIM_OK) {
        status = control_quality_analyze_step(
            samples,
            sample_count,
            event_time_s,
            target,
            criteria.settling_band_fraction,
            &reversal_metrics);
    }
    if (status == SIM_OK) {
        status = run_maneuver(
            MANEUVER_RECOVERY,
            &controller_config,
            &criteria,
            samples,
            sample_count,
            &event_time_s,
            &target);
    }
    if (status == SIM_OK) {
        status = control_quality_analyze_step(
            samples,
            sample_count,
            event_time_s,
            target,
            criteria.settling_band_fraction,
            &recovery_metrics);
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "control_quality_report: analysis failed: %s\n", sim_status_to_string(status));
        return 1;
    }
    step_pass = metrics_pass(
        &step_metrics,
        &criteria,
        criteria.max_step_settling_time_s,
        controller_config.guidance.max_accel_rate_mps3);
    reversal_pass = metrics_pass(
        &reversal_metrics,
        &criteria,
        criteria.max_reversal_settling_time_s,
        controller_config.guidance.max_accel_rate_mps3);
    recovery_pass = metrics_pass(
        &recovery_metrics,
        &criteria,
        criteria.max_recovery_settling_time_s,
        controller_config.guidance.max_accel_rate_mps3);
    if (options.output_path != 0) {
        output = fopen(options.output_path, "wb");
        if (output == 0) {
            (void)fprintf(stderr, "control_quality_report: cannot open output\n");
            return 1;
        }
    }
    (void)fprintf(output, "{\n");
    (void)fprintf(output, "  \"schema_version\": 1,\n");
    (void)fprintf(output, "  \"scope\": \"engineering_sil_baseline_not_model_fidelity_validation\",\n");
    (void)fprintf(output, "  \"flight_control_config\": \"%s\",\n", options.flight_control_path);
    (void)fprintf(output, "  \"criteria_config\": \"%s\",\n", options.criteria_path);
    (void)fprintf(
        output,
        "  \"pass\": %s,\n",
        step_pass != 0 && reversal_pass != 0 && recovery_pass != 0 ? "true" : "false");
    (void)fprintf(output, "  \"maneuvers\": {\n");
    write_metrics(output, "acceleration_step", &step_metrics, step_pass, 1);
    write_metrics(output, "command_reversal", &reversal_metrics, reversal_pass, 1);
    write_metrics(output, "dropout_recovery", &recovery_metrics, recovery_pass, 0);
    (void)fprintf(output, "  }\n");
    (void)fprintf(output, "}\n");
    if (options.output_path != 0 && fclose(output) != 0) {
        return 1;
    }
    return step_pass != 0 && reversal_pass != 0 && recovery_pass != 0 ? 0 : 1;
}
