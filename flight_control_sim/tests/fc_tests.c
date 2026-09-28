/** @file fc_tests.c
 *  @brief P6 飞控状态机、制导和指令保护单元测试。
 */
#include "common/protocol.h"
#include "common/vec3.h"
#include "fc/autopilot.h"
#include "fc/command_manager.h"
#include "fc/fc_health.h"
#include "fc/fc_modes.h"
#include "fc/fc_state.h"
#include "fc/guidance_png.h"
#include "fc/safety_monitor.h"

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
    config.autopilot.enable_attitude_loop = 1;
    config.autopilot.enable_control_allocation = 1;
    config.autopilot.max_attitude_cmd_rad = 0.35;
    config.autopilot.max_body_rate_cmd_radps = 1.0;
    config.autopilot.attitude_time_constant_s = 0.25;
    config.autopilot.gyro_damping_gain = 0.2;
    config.autopilot.fin_accel_effectiveness_mps2_per_rad = 150.0;
    config.autopilot.max_fin_deflection_rad = 0.35;
    config.safety.sensor_timeout_s = 0.1;
    config.safety.command_hold_s = 0.2;
    config.safety.reject_nan = 1;
    config.safety.reject_old_seq = 1;
    config.safety.max_consecutive_bad_frames = 3u;
    config.scheduler_base_rate_hz = 100.0;
    return config;
}

/** @brief 向测试控制器配置写入一个调度任务。 */
static void add_test_task(FlightControllerConfig *config, const char *name, uint32_t period_ticks)
{
    FcTask *task = &config->scheduler_tasks[config->scheduler_task_count++];

    (void)snprintf(task->name, sizeof(task->name), "%s", name);
    task->period_ticks = period_ticks;
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

    input.closing_velocity_mps = 100.0;
    input.los_rate_ecef = vec3_make(0.0, 0.0, 10.0);
    config.max_accel_mps2 = 50.0;
    failures += expect_int(guidance_png_update(&config, &input, &output) == SIM_OK, "png_limit_ok");
    failures += expect_near(vec3_norm(output.accel_cmd_ecef), 50.0, 1.0e-12, "png_limit_norm");
    failures += expect_int(output.accel_cmd_ecef.y > 0.0, "png_limit_direction");

    input.range_m = NAN;
    failures += expect_int(
        guidance_png_update(&config, &input, &output) == SIM_ERR_NUMERIC,
        "png_reject_nan_range");
    input.range_m = 1000.0;
    input.closing_velocity_mps = -1.0;
    failures += expect_int(
        guidance_png_update(&config, &input, &output) == SIM_ERR_OUT_OF_RANGE,
        "png_reject_negative_closing");
    return failures;
}

/** @brief 验证自动驾驶仪生成姿态、角速度和舵面分配命令。 */
static int test_autopilot_attitude_allocation(void)
{
    int failures = 0;
    Autopilot autopilot;
    AutopilotConfig config;
    NavState nav;
    GuidancePngOutput guidance;
    AutopilotCommand command;

    (void)memset(&config, 0, sizeof(config));
    config.enable_attitude_loop = 1;
    config.enable_control_allocation = 1;
    config.max_attitude_cmd_rad = 0.35;
    config.max_body_rate_cmd_radps = 1.0;
    config.attitude_time_constant_s = 0.25;
    config.gyro_damping_gain = 0.2;
    config.fin_accel_effectiveness_mps2_per_rad = 100.0;
    config.max_fin_deflection_rad = 0.2;
    failures += expect_int(autopilot_init(&autopilot, &config) == SIM_OK, "autopilot_init");

    (void)memset(&nav, 0, sizeof(nav));
    nav.missile_vel_ecef_est = vec3_make(300.0, 0.0, 0.0);
    nav.omega_b_est = vec3_make(0.0, 0.1, -0.1);
    nav.lat_rad = 0.0;
    nav.lon_rad = 0.0;
    nav.valid_flags = FC_NAV_VALID_KINEMATICS | FC_NAV_VALID_GEODETIC;
    guidance.accel_cmd_ecef = vec3_make(0.0, 20.0, 30.0);
    failures += expect_int(
        autopilot_update(&autopilot, &nav, &guidance, &command) == SIM_OK,
        "autopilot_update");
    failures += expect_int(command.attitude_cmd.y > 0.0, "autopilot_pitch_cmd");
    failures += expect_int(command.attitude_cmd.z > 0.0, "autopilot_yaw_cmd");
    failures += expect_int(command.body_rate_cmd.y > 0.0, "autopilot_pitch_rate_cmd");
    failures += expect_int(command.body_rate_cmd.z > 0.0, "autopilot_yaw_rate_cmd");
    failures += expect_near(command.actuator_cmd[0], 0.2, 1.0e-12, "autopilot_pitch_fin_limit");
    failures += expect_near(command.actuator_cmd[1], 0.2, 1.0e-12, "autopilot_yaw_fin_limit");
    return failures;
}

/** @brief 验证自动驾驶仪关闭内环时只透传加速度命令。 */
static int test_autopilot_disabled_inner_loops(void)
{
    int failures = 0;
    Autopilot autopilot;
    AutopilotConfig config;
    GuidancePngOutput guidance;
    AutopilotCommand command;

    (void)memset(&config, 0, sizeof(config));
    config.enable_attitude_loop = 0;
    config.enable_control_allocation = 0;
    config.max_attitude_cmd_rad = 0.35;
    config.max_body_rate_cmd_radps = 1.0;
    config.attitude_time_constant_s = 0.25;
    config.gyro_damping_gain = 0.2;
    config.fin_accel_effectiveness_mps2_per_rad = 100.0;
    config.max_fin_deflection_rad = 0.2;
    guidance.accel_cmd_ecef = vec3_make(1.0, 2.0, 3.0);

    failures += expect_int(autopilot_init(&autopilot, &config) == SIM_OK, "autopilot_disabled_init");
    failures += expect_int(
        autopilot_update(&autopilot, 0, &guidance, &command) == SIM_OK,
        "autopilot_disabled_update");
    failures += expect_near(command.accel_cmd_ecef.x, 1.0, 1.0e-12, "autopilot_disabled_accel_x");
    failures += expect_near(command.accel_cmd_ecef.y, 2.0, 1.0e-12, "autopilot_disabled_accel_y");
    failures += expect_near(command.accel_cmd_ecef.z, 3.0, 1.0e-12, "autopilot_disabled_accel_z");
    failures += expect_near(vec3_norm(command.attitude_cmd), 0.0, 1.0e-12, "autopilot_disabled_att");
    failures += expect_near(vec3_norm(command.body_rate_cmd), 0.0, 1.0e-12, "autopilot_disabled_rate");
    failures += expect_near(command.actuator_cmd[0], 0.0, 1.0e-12, "autopilot_disabled_fin0");
    failures += expect_near(command.actuator_cmd[1], 0.0, 1.0e-12, "autopilot_disabled_fin1");
    failures += expect_int(autopilot.accepted_count == 1u, "autopilot_disabled_accepted");
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

/** @brief 验证命令保持窗口按最后一条新鲜命令计时并最终降为受控零。 */
static int test_command_manager_hold_timeout(void)
{
    int failures = 0;
    CommandManager manager;
    CommandManagerConfig config = { 1000.0, 10000.0, 0.2 };
    AutopilotCommand request;
    ControlCommand command;
    uint32_t flags = 0u;

    failures += expect_int(command_manager_init(&manager, &config) == SIM_OK, "hold_timeout_init");
    autopilot_zero_command(&request);
    request.accel_cmd_ecef = vec3_make(10.0, 0.0, 0.0);
    failures += expect_int(
        command_manager_build(
            &manager,
            1u,
            1.0,
            0.01,
            FC_GUIDANCE_ACTIVE,
            0u,
            0,
            &request,
            &command,
            &flags) == SIM_OK,
        "hold_timeout_fresh");
    failures += expect_int(
        command_manager_build(
            &manager,
            2u,
            1.1,
            0.01,
            FC_COMMAND_HOLD,
            0u,
            1,
            0,
            &command,
            &flags) == SIM_OK,
        "hold_timeout_within_window");
    failures += expect_near(command.accel_cmd_ecef.x, 10.0, 1.0e-12, "hold_timeout_held_value");
    failures += expect_int(
        command_manager_build(
            &manager,
            3u,
            1.21,
            0.01,
            FC_COMMAND_HOLD,
            0u,
            1,
            0,
            &command,
            &flags) == SIM_OK,
        "hold_timeout_expired");
    failures += expect_near(vec3_norm(command.accel_cmd_ecef), 0.0, 1.0e-12, "hold_timeout_zero");
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

/** @brief 验证安全监视器在通信恢复后的坏帧计数和接受状态恢复。 */
static int test_safety_monitor_recovery(void)
{
    int failures = 0;
    FcSafetyConfig config = { 0.1, 0.2, 1, 1, 3u };
    SafetyMonitor monitor;
    SafetyAssessment assessment;
    SensorFrame sensor;

    failures += expect_int(safety_monitor_init(&monitor, &config) == SIM_OK, "safety_recovery_init");
    sensor = make_sensor(1u, 0.01);
    failures += expect_int(
        safety_monitor_check_sensor(&monitor, &sensor, 0, 0u, 0, 0.0, &assessment) == SIM_OK,
        "safety_recovery_initial_check");
    failures += expect_int(assessment.accept_frame != 0, "safety_recovery_initial_accept");
    failures += expect_int(monitor.consecutive_bad_frames == 0u, "safety_recovery_initial_count");

    sensor = make_sensor(2u, 0.02);
    sensor.sensor_valid_flags &= ~SIM_SENSOR_VALID_SEEKER;
    failures += expect_int(
        safety_monitor_check_sensor(&monitor, &sensor, 1, 1u, 1, 0.01, &assessment) == SIM_OK,
        "safety_recovery_invalid_check");
    failures += expect_int(assessment.accept_frame != 0, "safety_recovery_invalid_still_usable");
    failures += expect_int(assessment.request_hold != 0, "safety_recovery_invalid_hold");
    failures += expect_int(monitor.consecutive_bad_frames == 1u, "safety_recovery_invalid_count");

    sensor = make_sensor(1u, 0.03);
    failures += expect_int(
        safety_monitor_check_sensor(&monitor, &sensor, 1, 2u, 1, 0.02, &assessment) == SIM_OK,
        "safety_recovery_old_check");
    failures += expect_int(assessment.accept_frame == 0, "safety_recovery_old_reject");
    failures += expect_int(assessment.request_hold != 0, "safety_recovery_old_hold");
    failures += expect_int(monitor.consecutive_bad_frames == 2u, "safety_recovery_old_count");

    sensor = make_sensor(3u, 0.04);
    failures += expect_int(
        safety_monitor_check_sensor(&monitor, &sensor, 1, 2u, 1, 0.03, &assessment) == SIM_OK,
        "safety_recovery_valid_check");
    failures += expect_int(assessment.accept_frame != 0, "safety_recovery_valid_accept");
    failures += expect_int(assessment.request_hold == 0, "safety_recovery_valid_no_hold");
    failures += expect_int(monitor.consecutive_bad_frames == 0u, "safety_recovery_valid_count_reset");
    return failures;
}

/** @brief 验证控制器按 scheduler.tasks[] 周期复用导航缓存。 */
static int test_flight_controller_multirate_scheduler(void)
{
    int failures = 0;
    FlightController controller;
    FlightControllerConfig config = make_controller_config();
    ControlCommand command;
    SensorFrame sensor;

    add_test_task(&config, "receive", 1u);
    add_test_task(&config, "navigation", 2u);
    add_test_task(&config, "guidance", 1u);
    add_test_task(&config, "controller", 1u);
    add_test_task(&config, "safety", 1u);
    failures += expect_int(
        flight_controller_init(&controller, &config) == SIM_OK,
        "multirate_init");
    sensor = make_sensor(1u, 0.01);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "multirate_step_0");
    failures += expect_int(controller.estimator.accepted_frames == 1u, "multirate_nav_first");
    sensor = make_sensor(2u, 0.02);
    sensor.missile_vel_ecef_meas = vec3_make(100.0, 200.0, 0.0);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "multirate_step_1");
    failures += expect_int(controller.estimator.accepted_frames == 1u, "multirate_nav_hold");
    sensor = make_sensor(3u, 0.03);
    failures += expect_int(
        flight_controller_step(&controller, &sensor, &command) == SIM_OK,
        "multirate_step_2");
    failures += expect_int(controller.estimator.accepted_frames == 2u, "multirate_nav_second");
    return failures;
}

/** @brief 验证连续闭环控制输出满足限幅、速率和舵偏边界。 */
static int test_flight_controller_control_quality_bounds(void)
{
    int failures = 0;
    FlightController controller;
    FlightControllerConfig config = make_controller_config();
    ControlCommand command;
    Vec3 previous_accel = vec3_make(0.0, 0.0, 0.0);
    int have_previous = 0;
    uint32_t index;

    failures += expect_int(
        flight_controller_init(&controller, &config) == SIM_OK,
        "quality_init");
    for (index = 1u; index <= 60u; ++index) {
        SensorFrame sensor = make_sensor(index, 0.01 * (double)index);
        const double sign = (index % 2u) == 0u ? 1.0 : -1.0;
        const double phase = (double)(index % 7u);
        Vec3 delta;

        sensor.target_los_rate_ecef_meas = vec3_make(
            0.0,
            sign * (0.02 + (0.01 * phase)),
            0.20 - (0.004 * (double)index));
        failures += expect_int(
            flight_controller_step(&controller, &sensor, &command) == SIM_OK,
            "quality_step_ok");
        failures += expect_int(vec3_isfinite(command.accel_cmd_ecef), "quality_accel_finite");
        failures += expect_int(vec3_isfinite(command.attitude_cmd), "quality_attitude_finite");
        failures += expect_int(vec3_isfinite(command.body_rate_cmd), "quality_rate_finite");
        failures += expect_int(
            vec3_norm(command.accel_cmd_ecef) <= config.guidance.max_accel_mps2 + 1.0e-9,
            "quality_accel_norm_limit");
        failures += expect_int(
            fabs(command.actuator_cmd[0]) <= config.autopilot.max_fin_deflection_rad + 1.0e-12 &&
                fabs(command.actuator_cmd[1]) <= config.autopilot.max_fin_deflection_rad + 1.0e-12,
            "quality_fin_limits");
        if (have_previous != 0) {
            delta = vec3_sub(command.accel_cmd_ecef, previous_accel);
            failures += expect_int(
                vec3_norm(delta) <=
                    (config.guidance.max_accel_rate_mps3 * sensor.dt) + 1.0e-9,
                "quality_accel_rate_limit");
        }
        previous_accel = command.accel_cmd_ecef;
        have_previous = 1;
    }
    failures += expect_int(controller.autopilot.accepted_count > 0u, "quality_autopilot_active");
    return failures;
}

int main(void)
{
    int failures = 0;

    failures += test_guidance_png_direction();
    failures += test_autopilot_attitude_allocation();
    failures += test_autopilot_disabled_inner_loops();
    failures += test_command_manager_limits();
    failures += test_command_manager_hold_timeout();
    failures += test_mode_state_machine();
    failures += test_flight_controller_protection();
    failures += test_safety_monitor_recovery();
    failures += test_flight_controller_multirate_scheduler();
    failures += test_flight_controller_control_quality_bounds();
    return failures == 0 ? 0 : 1;
}
