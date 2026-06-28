/** @file fc_tests.c
 *  @brief P6 飞控状态机、制导和指令保护单元测试。
 */
#include "common/protocol.h"
#include "common/vec3.h"
#include "fc/command_manager.h"
#include "fc/fc_health.h"
#include "fc/fc_modes.h"
#include "fc/fc_state.h"
#include "fc/guidance_png.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief 记录布尔断言结果并返回失败计数增量。 */
static int expect_int(int condition, const char *name)
{
    if (!condition) {
        (void)fprintf(stderr, "failed: %s\n", name);
        return 1;
    }
    return 0;
}

/** @brief 使用绝对误差比较双精度值。 */
static int expect_near(double actual, double expected, double tolerance, const char *name)
{
    return expect_int(fabs(actual - expected) <= tolerance, name);
}

/** @brief 构造一帧默认有效的导引头传感器数据。 */
static SensorFrame make_sensor(uint32_t seq, double sim_time)
{
    SensorFrame sensor;

    (void)memset(&sensor, 0, sizeof(sensor));
    sensor.seq = seq;
    sensor.sim_time = sim_time;
    sensor.dt = 0.01;
    sensor.missile_vel_ecef_meas = vec3_make(300.0, 0.0, 0.0);
    sensor.missile_accel_ecef_meas = vec3_make(0.0, 0.0, 0.0);
    sensor.missile_gyro_b_meas = vec3_make(0.0, 0.0, 0.0);
    sensor.missile_lat_rad_meas = 0.5;
    sensor.missile_lon_rad_meas = 2.0;
    sensor.missile_height_m_meas = 1000.0;
    sensor.missile_height_agl_m_meas = 1000.0;
    sensor.target_range_meas = 1000.0;
    sensor.target_los_unit_ecef_meas = vec3_make(1.0, 0.0, 0.0);
    sensor.target_los_rate_ecef_meas = vec3_make(0.0, 0.0, 0.1);
    sensor.target_closing_velocity_meas = 100.0;
    sensor.sensor_valid_flags =
        SIM_SENSOR_VALID_SEEKER |
        SIM_SENSOR_VALID_IMU_GYRO |
        SIM_SENSOR_VALID_ACCEL |
        SIM_SENSOR_VALID_SPEED |
        SIM_SENSOR_VALID_GEODETIC;
    return sensor;
}

/** @brief 构造可复用的控制器配置。 */
static FlightControllerConfig make_controller_config(void)
{
    FlightControllerConfig config;

    (void)memset(&config, 0, sizeof(config));
    config.guidance.navigation_constant = 4.0;
    config.guidance.max_accel_mps2 = 350.0;
    config.guidance.max_accel_rate_mps3 = 2000.0;
    config.safety.sensor_timeout_s = 0.1;
    config.safety.command_hold_s = 0.2;
    config.safety.reject_nan = 1;
    config.safety.reject_old_seq = 1;
    config.safety.max_consecutive_bad_frames = 3u;
    config.scheduler_base_rate_hz = 100.0;
    return config;
}

/** @brief 验证三维 PNG 的叉乘方向和限幅方向。 */
static int test_guidance_png_direction(void)
{
    int failures = 0;
    GuidancePngConfig config = { 4.0, 1000.0, 100.0 };
    GuidancePngInput input;
    GuidancePngOutput output;

    input.range_m = 1000.0;
    input.closing_velocity_mps = 100.0;
    input.los_unit_ecef = vec3_make(1.0, 0.0, 0.0);
    input.los_rate_ecef = vec3_make(0.0, 0.0, 0.1);
    failures += expect_int(guidance_png_update(&config, &input, &output) == SIM_OK, "png_ok");
    failures += expect_near(output.accel_cmd_ecef.x, 0.0, 1.0e-12, "png_x");
    failures += expect_near(output.accel_cmd_ecef.y, 40.0, 1.0e-12, "png_y_direction");
    failures += expect_near(output.accel_cmd_ecef.z, 0.0, 1.0e-12, "png_z");

    input.closing_velocity_mps = -1.0;
    failures += expect_int(
        guidance_png_update(&config, &input, &output) == SIM_ERR_OUT_OF_RANGE,
        "png_reject_negative_closing");
    return failures;
}

/** @brief 验证命令范数限幅和变化率限制。 */
static int test_command_manager_limits(void)
{
    int failures = 0;
    CommandManager manager;
    CommandManagerConfig config = { 1000.0, 10.0, 0.2 };
    AutopilotCommand request;
    ControlCommand command;
    uint32_t flags = 0u;

    failures += expect_int(command_manager_init(&manager, &config) == SIM_OK, "cmd_init");
    autopilot_zero_command(&request);
    request.accel_cmd_ecef = vec3_make(100.0, 0.0, 0.0);
    failures += expect_int(
        command_manager_build(
            &manager,
            1u,
            0.5,
            0.5,
            FC_GUIDANCE_ACTIVE,
            0u,
            0,
            &request,
            &command,
            &flags) == SIM_OK,
        "cmd_build");
    failures += expect_near(command.accel_cmd_ecef.x, 5.0, 1.0e-12, "cmd_rate_limit");
    failures += expect_int((flags & FC_HEALTH_WARNING_RATE_LIMITED) != 0u, "cmd_rate_flag");

    request.accel_cmd_ecef = vec3_make(5000.0, 0.0, 0.0);
    failures += expect_int(
        command_manager_build(
            &manager,
            2u,
            1.0,
            0.5,
            FC_GUIDANCE_ACTIVE,
            0u,
            0,
            &request,
            &command,
            &flags) == SIM_OK,
        "cmd_norm_build");
    failures += expect_int((flags & FC_HEALTH_WARNING_COMMAND_LIMITED) != 0u, "cmd_norm_flag");
    return failures;
}

/** @brief 验证模式状态机从上电到制导激活的转换。 */
static int test_mode_state_machine(void)
{
    int failures = 0;
    FcModeInput input;
    FcMode mode = FC_POWER_ON;

    (void)memset(&input, 0, sizeof(input));
    input.self_test_ok = 1;
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_SELF_TEST, "mode_power_to_selftest");
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_WAIT_SENSOR, "mode_selftest_to_wait");
    input.sensor_frame_valid = 1;
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_NAV_READY, "mode_wait_to_nav");
    input.navigation_valid = 1;
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_GUIDANCE_STANDBY, "mode_nav_to_standby");
    input.guidance_valid = 1;
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_GUIDANCE_ACTIVE, "mode_standby_to_active");
    input.guidance_valid = 0;
    input.command_hold = 1;
    mode = fc_mode_next(mode, &input);
    failures += expect_int(mode == FC_COMMAND_HOLD, "mode_active_to_hold");
    return failures;
}

/** @brief 验证完整飞控控制器的无效测量和旧帧保护。 */
static int test_flight_controller_protection(void)
{
    int failures = 0;
    FlightController controller;
    FlightControllerConfig config = make_controller_config();
    SensorFrame sensor = make_sensor(1u, 0.01);
    ControlCommand command;

    failures += expect_int(
        flight_controller_init(&controller, &config) == SIM_OK,
        "controller_init");

    sensor.sensor_valid_flags &= ~SIM_SENSOR_VALID_SEEKER;
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "controller_invalid_seeker");
    failures += expect_int(command.command_mode == (uint32_t)FC_COMMAND_HOLD, "controller_hold_mode");
    failures += expect_int(
        (command.command_status & FC_HEALTH_WARNING_MEASUREMENT_INVALID) != 0u,
        "controller_invalid_status");
    failures += expect_near(vec3_norm(command.accel_cmd_ecef), 0.0, 1.0e-12, "controller_zero_hold");

    sensor = make_sensor(2u, 0.02);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "controller_valid");
    failures += expect_int(command.command_mode == (uint32_t)FC_GUIDANCE_ACTIVE, "controller_active");
    failures += expect_int(
        (command.command_status & FC_HEALTH_WARNING_RATE_LIMITED) != 0u,
        "controller_rate_limited");

    sensor = make_sensor(2u, 0.03);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "controller_old_frame");
    failures += expect_int(
        (command.command_status & FC_HEALTH_WARNING_OLD_FRAME) != 0u,
        "controller_old_frame_status");

    sensor = make_sensor(3u, 0.50);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "controller_timeout");
    failures += expect_int(
        (command.command_status & FC_HEALTH_WARNING_SENSOR_TIMEOUT) != 0u,
        "controller_timeout_status");
    failures += expect_int(command.command_mode == (uint32_t)FC_DEGRADED, "controller_timeout_mode");

    failures += expect_int(
        flight_controller_init(&controller, &config) == SIM_OK,
        "controller_reinit");
    sensor = make_sensor(1u, 0.01);
    sensor.target_range_meas = NAN;
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "controller_nan_frame");
    failures += expect_int(
        (command.command_status & FC_HEALTH_FAULT_NUMERIC) != 0u,
        "controller_nan_status");
    failures += expect_int(command.command_mode == (uint32_t)FC_FAULT, "controller_nan_mode");
    return failures;
}

int main(void)
{
    int failures = 0;

    failures += test_guidance_png_direction();
    failures += test_command_manager_limits();
    failures += test_mode_state_machine();
    failures += test_flight_controller_protection();
    return failures == 0 ? 0 : 1;
}
