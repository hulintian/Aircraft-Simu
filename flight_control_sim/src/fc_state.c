/** @file fc_state.c
 *  @brief 单实例飞控控制器实现。
 */
#include "fc/fc_state.h"

#include <string.h>

/** @brief 按当前单帧判定快速推进启动和工作模式。 */
static FcMode advance_mode(FcMode current, const FcModeInput *input)
{
    FcMode mode = current;
    unsigned int iteration;

    for (iteration = 0u; iteration < 6u; ++iteration) {
        const FcMode next = fc_mode_next(mode, input);

        if (next == mode) {
            break;
        }
        mode = next;
        if (mode == FC_GUIDANCE_ACTIVE ||
            mode == FC_COMMAND_HOLD ||
            mode == FC_DEGRADED ||
            mode == FC_FAULT ||
            mode == FC_SHUTDOWN) {
            break;
        }
    }
    return mode;
}

SimStatus flight_controller_init(
    FlightController *controller,
    const FlightControllerConfig *config)
{
    CommandManagerConfig command_config;
    SimStatus status;

    if (controller == 0 || config == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(controller, 0, sizeof(*controller));
    controller->mode = FC_POWER_ON;
    status = fc_scheduler_init(&controller->scheduler, config->scheduler_base_rate_hz);
    if (status == SIM_OK) {
        status = fc_scheduler_add_task(&controller->scheduler, "receive", 1u);
    }
    if (status == SIM_OK) {
        status = fc_scheduler_add_task(&controller->scheduler, "navigation", 1u);
    }
    if (status == SIM_OK) {
        status = fc_scheduler_add_task(&controller->scheduler, "guidance", 1u);
    }
    if (status == SIM_OK) {
        status = fc_scheduler_add_task(&controller->scheduler, "controller", 1u);
    }
    if (status == SIM_OK) {
        status = fc_scheduler_add_task(&controller->scheduler, "safety", 1u);
    }
    if (status == SIM_OK) {
        status = estimator_init(&controller->estimator);
    }
    if (status == SIM_OK) {
        status = guidance_manager_init(&controller->guidance, &config->guidance);
    }
    if (status == SIM_OK) {
        status = autopilot_init(&controller->autopilot);
    }
    if (status == SIM_OK) {
        status = safety_monitor_init(&controller->safety, &config->safety);
    }
    if (status == SIM_OK) {
        command_config.max_accel_mps2 = config->guidance.max_accel_mps2;
        command_config.max_accel_rate_mps3 = config->guidance.max_accel_rate_mps3;
        command_config.command_hold_s = config->safety.command_hold_s;
        status = command_manager_init(&controller->command_manager, &command_config);
    }
    return status;
}

SimStatus flight_controller_step(
    FlightController *controller,
    const SensorFrame *sensor,
    ControlCommand *command)
{
    SafetyAssessment assessment;
    NavState nav;
    GuidancePngOutput guidance;
    AutopilotCommand autopilot_command;
    FcModeInput mode_input;
    uint32_t status_flags;
    int guidance_valid = 0;
    int nav_valid = 0;
    int request_hold = 0;
    SimStatus status;

    if (controller == 0 || sensor == 0 || command == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    fc_health_reset(&controller->health);
    status = safety_monitor_check_sensor(
        &controller->safety,
        sensor,
        controller->have_seq,
        controller->last_seq,
        controller->have_last_sensor_time,
        controller->last_sensor_time,
        &assessment);
    if (status != SIM_OK) {
        return status;
    }
    status_flags = assessment.status_flags;
    request_hold = assessment.request_hold;
    autopilot_zero_command(&autopilot_command);
    (void)memset(&nav, 0, sizeof(nav));
    (void)memset(&guidance, 0, sizeof(guidance));

    if (assessment.accept_frame != 0) {
        status = estimator_update(&controller->estimator, sensor, &nav);
        if (status == SIM_OK) {
            nav_valid = (nav.valid_flags & FC_NAV_VALID_KINEMATICS) != 0u;
            if (navigation_has_guidance_solution(&nav)) {
                status = guidance_manager_update(&controller->guidance, &nav, &guidance);
                if (status == SIM_OK) {
                    status = autopilot_update(&controller->autopilot, &guidance, &autopilot_command);
                    guidance_valid = status == SIM_OK;
                }
            }
        }
        if (status != SIM_OK) {
            if (status == SIM_ERR_NUMERIC) {
                status_flags |= FC_HEALTH_FAULT_NUMERIC;
            } else {
                status_flags |= FC_HEALTH_WARNING_CLOSING_VELOCITY;
            }
            request_hold = 1;
        }
    }

    (void)memset(&mode_input, 0, sizeof(mode_input));
    mode_input.self_test_ok = 1;
    mode_input.sensor_frame_valid = assessment.accept_frame;
    mode_input.navigation_valid = nav_valid;
    mode_input.guidance_valid = guidance_valid;
    mode_input.command_hold = request_hold;
    mode_input.degraded = assessment.degraded;
    mode_input.fault = assessment.fault || ((status_flags & FC_HEALTH_FAULT_NUMERIC) != 0u);
    controller->mode = advance_mode(controller->mode, &mode_input);

    if ((status_flags & FC_HEALTH_FAULT_NUMERIC) != 0u) {
        fc_health_set_fault(&controller->health, status_flags & FC_HEALTH_FAULT_NUMERIC);
    }
    fc_health_set_warning(&controller->health, status_flags & ~FC_HEALTH_FAULT_NUMERIC);
    status_flags = fc_health_command_status(&controller->health);

    status = command_manager_build(
        &controller->command_manager,
        sensor->seq,
        sensor->sim_time,
        sensor->dt,
        controller->mode,
        status_flags,
        request_hold || (guidance_valid == 0),
        guidance_valid != 0 ? &autopilot_command : 0,
        command,
        &status_flags);
    if (status != SIM_OK) {
        return status;
    }
    command->command_mode = (uint32_t)controller->mode;
    command->command_status = status_flags;
    controller->state.last_sensor = *sensor;
    controller->state.last_command = *command;
    if (assessment.accept_frame != 0) {
        controller->last_seq = sensor->seq;
        controller->have_seq = 1;
        controller->last_sensor_time = sensor->sim_time;
        controller->have_last_sensor_time = 1;
    }
    fc_scheduler_advance(&controller->scheduler);
    return SIM_OK;
}
