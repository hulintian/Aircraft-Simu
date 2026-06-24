/** @file fault_injection.c
 *  @brief 环境程序故障脚本解析和运行期应用实现。
 */
#include "env/fault_injection.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief 构造 `faults[index].field` 路径。 */
static SimStatus make_fault_path(
    size_t index,
    const char *field,
    char *path,
    size_t path_size)
{
    int written;

    if (field == 0 || path == 0 || path_size == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    written = snprintf(path, path_size, "faults[%lu].%s", (unsigned long)index, field);
    if (written < 0 || (size_t)written >= path_size) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    return SIM_OK;
}

/** @brief 可选读取字符串字段。 */
static SimStatus get_optional_string(
    const ConfigTree *config,
    size_t index,
    const char *field,
    char *out,
    size_t out_size,
    int *found)
{
    char path[128];
    SimStatus status;

    status = make_fault_path(index, field, path, sizeof(path));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(config, path, out, out_size);
    if (status == SIM_OK) {
        if (found != 0) {
            *found = 1;
        }
        return SIM_OK;
    }
    if (found != 0) {
        *found = 0;
    }
    return SIM_OK;
}

/** @brief 必须读取字符串字段。 */
static SimStatus get_required_string(
    const ConfigTree *config,
    size_t index,
    const char *field,
    char *out,
    size_t out_size)
{
    char path[128];
    SimStatus status;

    status = make_fault_path(index, field, path, sizeof(path));
    if (status != SIM_OK) {
        return status;
    }
    return config_get_string(config, path, out, out_size);
}

/** @brief 可选读取双精度字段。 */
static SimStatus get_optional_double(
    const ConfigTree *config,
    size_t index,
    const char *field,
    double *out,
    int *found)
{
    char path[128];
    SimStatus status;

    status = make_fault_path(index, field, path, sizeof(path));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, path, out);
    if (status == SIM_OK) {
        if (found != 0) {
            *found = 1;
        }
        return SIM_OK;
    }
    if (found != 0) {
        *found = 0;
    }
    return SIM_OK;
}

/** @brief 可选读取布尔字段。 */
static SimStatus get_optional_bool(
    const ConfigTree *config,
    size_t index,
    const char *field,
    int *out,
    int *found)
{
    char path[128];
    SimStatus status;

    status = make_fault_path(index, field, path, sizeof(path));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_bool(config, path, out);
    if (status == SIM_OK) {
        if (found != 0) {
            *found = 1;
        }
        return SIM_OK;
    }
    if (found != 0) {
        *found = 0;
    }
    return SIM_OK;
}

/** @brief 可选读取三轴字段。 */
static SimStatus get_optional_vec3(
    const ConfigTree *config,
    size_t index,
    const char *field,
    Vec3 *out,
    int *found)
{
    char path[128];
    double values[3];
    SimStatus status;

    status = make_fault_path(index, field, path, sizeof(path));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double_array(config, path, values, 3u);
    if (status == SIM_OK) {
        *out = vec3_make(values[0], values[1], values[2]);
        if (found != 0) {
            *found = 1;
        }
        return SIM_OK;
    }
    if (found != 0) {
        *found = 0;
    }
    return SIM_OK;
}

/** @brief 解析目标字符串。 */
static SimStatus parse_target(const char *text, FaultTarget *target)
{
    if (text == 0 || target == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strcmp(text, "sensor.seeker") == 0) {
        *target = FAULT_TARGET_SENSOR_SEEKER;
    } else if (strcmp(text, "sensor.seeker.range") == 0) {
        *target = FAULT_TARGET_SENSOR_SEEKER_RANGE;
    } else if (strcmp(text, "sensor.seeker.los_unit") == 0) {
        *target = FAULT_TARGET_SENSOR_SEEKER_LOS_UNIT;
    } else if (strcmp(text, "sensor.seeker.los_rate") == 0) {
        *target = FAULT_TARGET_SENSOR_SEEKER_LOS_RATE;
    } else if (strcmp(text, "sensor.seeker.closing_velocity") == 0) {
        *target = FAULT_TARGET_SENSOR_SEEKER_CLOSING_VELOCITY;
    } else if (strcmp(text, "sensor.imu") == 0 ||
        strcmp(text, "sensor.imu.gyro") == 0) {
        *target = FAULT_TARGET_SENSOR_IMU_GYRO;
    } else if (strcmp(text, "sensor.accelerometer") == 0 ||
        strcmp(text, "sensor.accel") == 0) {
        *target = FAULT_TARGET_SENSOR_ACCEL;
    } else if (strcmp(text, "sensor.speedometer") == 0 ||
        strcmp(text, "sensor.speed") == 0) {
        *target = FAULT_TARGET_SENSOR_SPEED;
    } else if (strcmp(text, "actuator.accel_x") == 0 ||
        strcmp(text, "actuator.acceleration[0]") == 0) {
        *target = FAULT_TARGET_ACTUATOR_ACCEL_X;
    } else if (strcmp(text, "actuator.accel_y") == 0 ||
        strcmp(text, "actuator.acceleration[1]") == 0) {
        *target = FAULT_TARGET_ACTUATOR_ACCEL_Y;
    } else if (strcmp(text, "actuator.accel_z") == 0 ||
        strcmp(text, "actuator.acceleration[2]") == 0) {
        *target = FAULT_TARGET_ACTUATOR_ACCEL_Z;
    } else {
        return SIM_ERR_CONFIG;
    }
    return SIM_OK;
}

/** @brief 解析故障类型字符串。 */
static SimStatus parse_type(const char *text, FaultType *type)
{
    if (text == 0 || type == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strcmp(text, "BIAS") == 0) {
        *type = FAULT_TYPE_BIAS;
    } else if (strcmp(text, "DROPOUT") == 0) {
        *type = FAULT_TYPE_DROPOUT;
    } else if (strcmp(text, "FORCE_INVALID") == 0 ||
        strcmp(text, "INVALID") == 0) {
        *type = FAULT_TYPE_FORCE_INVALID;
    } else if (strcmp(text, "STUCK") == 0) {
        *type = FAULT_TYPE_STUCK;
    } else if (strcmp(text, "SCALE") == 0) {
        *type = FAULT_TYPE_SCALE;
    } else {
        return SIM_ERR_CONFIG;
    }
    return SIM_OK;
}

/** @brief 返回目标对应的传感器有效位。 */
static uint32_t target_valid_mask(FaultTarget target)
{
    switch (target) {
    case FAULT_TARGET_SENSOR_SEEKER:
    case FAULT_TARGET_SENSOR_SEEKER_RANGE:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_UNIT:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_RATE:
    case FAULT_TARGET_SENSOR_SEEKER_CLOSING_VELOCITY:
        return SIM_SENSOR_VALID_SEEKER;
    case FAULT_TARGET_SENSOR_IMU_GYRO:
        return SIM_SENSOR_VALID_IMU_GYRO;
    case FAULT_TARGET_SENSOR_ACCEL:
        return SIM_SENSOR_VALID_ACCEL;
    case FAULT_TARGET_SENSOR_SPEED:
        return SIM_SENSOR_VALID_SPEED;
    default:
        return 0u;
    }
}

/** @brief 返回目标对应的传感器故障位。 */
static uint32_t target_fault_mask(FaultTarget target)
{
    switch (target) {
    case FAULT_TARGET_SENSOR_SEEKER:
    case FAULT_TARGET_SENSOR_SEEKER_RANGE:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_UNIT:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_RATE:
    case FAULT_TARGET_SENSOR_SEEKER_CLOSING_VELOCITY:
        return SIM_SENSOR_FAULT_SEEKER_DROPOUT;
    case FAULT_TARGET_SENSOR_IMU_GYRO:
        return SIM_SENSOR_FAULT_IMU_DROPOUT;
    case FAULT_TARGET_SENSOR_ACCEL:
        return SIM_SENSOR_FAULT_ACCEL_DROPOUT;
    case FAULT_TARGET_SENSOR_SPEED:
        return SIM_SENSOR_FAULT_SPEED_DROPOUT;
    default:
        return 0u;
    }
}

/** @brief 目标是否为虚拟执行机构。 */
static int target_actuator_index(FaultTarget target, size_t *index)
{
    if (index == 0) {
        return 0;
    }
    if (target == FAULT_TARGET_ACTUATOR_ACCEL_X) {
        *index = 0u;
        return 1;
    }
    if (target == FAULT_TARGET_ACTUATOR_ACCEL_Y) {
        *index = 1u;
        return 1;
    }
    if (target == FAULT_TARGET_ACTUATOR_ACCEL_Z) {
        *index = 2u;
        return 1;
    }
    return 0;
}

/** @brief 校验故障类型和目标对象是否匹配。 */
static int fault_type_matches_target(const FaultDefinition *fault)
{
    size_t actuator_index;

    if (fault == 0) {
        return 0;
    }
    if (fault->type == FAULT_TYPE_DROPOUT ||
        fault->type == FAULT_TYPE_FORCE_INVALID) {
        return target_valid_mask(fault->target) != 0u;
    }
    if (fault->type == FAULT_TYPE_STUCK ||
        fault->type == FAULT_TYPE_SCALE) {
        return target_actuator_index(fault->target, &actuator_index);
    }
    if (fault->type != FAULT_TYPE_BIAS) {
        return 0;
    }
    switch (fault->target) {
    case FAULT_TARGET_SENSOR_SEEKER_RANGE:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_UNIT:
    case FAULT_TARGET_SENSOR_SEEKER_LOS_RATE:
    case FAULT_TARGET_SENSOR_SEEKER_CLOSING_VELOCITY:
    case FAULT_TARGET_SENSOR_IMU_GYRO:
    case FAULT_TARGET_SENSOR_ACCEL:
    case FAULT_TARGET_SENSOR_SPEED:
        return 1;
    default:
        return 0;
    }
}

/** @brief 将单条激活故障累加到本步效果。 */
static SimStatus accumulate_fault(const FaultDefinition *fault, FaultStepEffects *effects)
{
    size_t actuator_index;

    if (fault == 0 || effects == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    ++effects->active_fault_count;
    if (fault->type == FAULT_TYPE_DROPOUT ||
        fault->type == FAULT_TYPE_FORCE_INVALID) {
        effects->sensor_valid_clear_mask |= target_valid_mask(fault->target);
        effects->sensor_fault_set_mask |= target_fault_mask(fault->target);
        return SIM_OK;
    }
    if (fault->type == FAULT_TYPE_STUCK) {
        if (!target_actuator_index(fault->target, &actuator_index)) {
            return SIM_ERR_CONFIG;
        }
        effects->actuator_stuck[actuator_index] = 1;
        return SIM_OK;
    }
    if (fault->type == FAULT_TYPE_SCALE) {
        if (!target_actuator_index(fault->target, &actuator_index)) {
            return SIM_ERR_CONFIG;
        }
        effects->actuator_command_scale[actuator_index] *= fault->scale;
        return SIM_OK;
    }
    if (fault->type != FAULT_TYPE_BIAS) {
        return SIM_ERR_CONFIG;
    }

    switch (fault->target) {
    case FAULT_TARGET_SENSOR_SEEKER_RANGE:
        effects->seeker_range_bias_m += fault->scalar_value;
        break;
    case FAULT_TARGET_SENSOR_SEEKER_LOS_UNIT:
        effects->seeker_los_unit_bias =
            vec3_add(effects->seeker_los_unit_bias, fault->vector_value);
        break;
    case FAULT_TARGET_SENSOR_SEEKER_LOS_RATE:
        effects->seeker_los_rate_bias_radps =
            vec3_add(effects->seeker_los_rate_bias_radps, fault->vector_value);
        break;
    case FAULT_TARGET_SENSOR_SEEKER_CLOSING_VELOCITY:
        effects->seeker_closing_velocity_bias_mps += fault->scalar_value;
        break;
    case FAULT_TARGET_SENSOR_IMU_GYRO:
        effects->gyro_bias_b_radps =
            vec3_add(effects->gyro_bias_b_radps, fault->vector_value);
        break;
    case FAULT_TARGET_SENSOR_ACCEL:
        effects->accel_bias_ecef_mps2 =
            vec3_add(effects->accel_bias_ecef_mps2, fault->vector_value);
        break;
    case FAULT_TARGET_SENSOR_SPEED:
        effects->speed_bias_ecef_mps =
            vec3_add(effects->speed_bias_ecef_mps, fault->vector_value);
        break;
    default:
        return SIM_ERR_CONFIG;
    }
    return SIM_OK;
}

/** @brief 判断当前时间是否处于故障激活窗口内。 */
static int fault_is_active(const FaultDefinition *fault, double sim_time_s)
{
    const double epsilon = 1.0e-12;

    if (fault == 0 || fault->enabled == 0) {
        return 0;
    }
    return sim_time_s + epsilon >= fault->start_time_s &&
        sim_time_s < fault->start_time_s + fault->duration_s - epsilon;
}

void fault_injection_init_empty(FaultInjection *faults)
{
    if (faults != 0) {
        (void)memset(faults, 0, sizeof(*faults));
    }
}

SimStatus fault_injection_load_config(const ConfigTree *config, FaultInjection *faults)
{
    size_t count;
    size_t index;
    SimStatus status;

    if (config == 0 || faults == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    fault_injection_init_empty(faults);
    status = config_validate_schema(config, 1u);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_array_count(config, "faults", &count);
    if (status != SIM_OK) {
        return status;
    }
    if (count > ENV_MAX_FAULTS) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    faults->enabled = 1;
    faults->fault_count = count;
    for (index = 0u; index < count; ++index) {
        FaultDefinition *fault = &faults->faults[index];
        double scalar = 0.0;
        Vec3 vector = vec3_make(0.0, 0.0, 0.0);
        int found = 0;

        fault->enabled = 1;
        (void)snprintf(fault->id, sizeof(fault->id), "fault_%02lu", (unsigned long)index);
        status = get_optional_string(
            config,
            index,
            "id",
            fault->id,
            sizeof(fault->id),
            &found);
        if (status != SIM_OK) {
            return status;
        }
        status = get_optional_bool(config, index, "enabled", &fault->enabled, &found);
        if (status != SIM_OK) {
            return status;
        }
        status = get_required_string(
            config,
            index,
            "target",
            fault->target_text,
            sizeof(fault->target_text));
        if (status != SIM_OK) {
            return status;
        }
        status = get_required_string(
            config,
            index,
            "type",
            fault->type_text,
            sizeof(fault->type_text));
        if (status != SIM_OK) {
            return status;
        }
        status = parse_target(fault->target_text, &fault->target);
        if (status == SIM_OK) {
            status = parse_type(fault->type_text, &fault->type);
        }
        if (status != SIM_OK) {
            return status;
        }

        status = get_optional_double(config, index, "start_time_s", &fault->start_time_s, &found);
        if (status != SIM_OK) {
            return status;
        }
        if (found == 0) {
            status = get_optional_double(config, index, "time_s", &fault->start_time_s, &found);
            if (status != SIM_OK || found == 0) {
                return status == SIM_OK ? SIM_ERR_CONFIG : status;
            }
        }
        status = get_optional_double(config, index, "duration_s", &fault->duration_s, &found);
        if (status != SIM_OK || found == 0) {
            return status == SIM_OK ? SIM_ERR_CONFIG : status;
        }
        status = get_optional_double(config, index, "value", &scalar, &found);
        if (status != SIM_OK) {
            return status;
        }
        fault->scalar_value = found != 0 ? scalar : 0.0;
        vector = vec3_make(fault->scalar_value, fault->scalar_value, fault->scalar_value);
        status = get_optional_vec3(config, index, "value_xyz", &vector, &found);
        if (status != SIM_OK) {
            return status;
        }
        fault->vector_value = vector;
        status = get_optional_double(config, index, "scale", &fault->scale, &found);
        if (status != SIM_OK) {
            return status;
        }
        if (found == 0) {
            fault->scale = fault->scalar_value;
        }
        if (!isfinite(fault->start_time_s) || fault->start_time_s < 0.0 ||
            !isfinite(fault->duration_s) || fault->duration_s <= 0.0 ||
            !isfinite(fault->scalar_value) ||
            !vec3_isfinite(fault->vector_value) ||
            !isfinite(fault->scale)) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        if (!fault_type_matches_target(fault)) {
            return SIM_ERR_CONFIG;
        }
    }
    return SIM_OK;
}

SimStatus fault_injection_update(
    FaultInjection *faults,
    double sim_time_s,
    FaultStepEffects *effects,
    FaultTransition *transitions,
    size_t transition_capacity,
    size_t *transition_count)
{
    size_t index;

    if (faults == 0 || effects == 0 || transition_count == 0 ||
        !isfinite(sim_time_s)) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(effects, 0, sizeof(*effects));
    effects->actuator_command_scale[0] = 1.0;
    effects->actuator_command_scale[1] = 1.0;
    effects->actuator_command_scale[2] = 1.0;
    *transition_count = 0u;
    if (faults->enabled == 0) {
        return SIM_OK;
    }
    for (index = 0u; index < faults->fault_count; ++index) {
        FaultDefinition *fault = &faults->faults[index];
        const int active = fault_is_active(fault, sim_time_s);

        if (active != fault->was_active) {
            if (transitions != 0 && *transition_count < transition_capacity) {
                FaultTransition *transition = &transitions[*transition_count];

                (void)memset(transition, 0, sizeof(*transition));
                (void)snprintf(transition->id, sizeof(transition->id), "%s", fault->id);
                (void)snprintf(
                    transition->target,
                    sizeof(transition->target),
                    "%s",
                    fault->target_text);
                (void)snprintf(
                    transition->type,
                    sizeof(transition->type),
                    "%s",
                    fault->type_text);
                transition->active = active;
            }
            ++(*transition_count);
            fault->was_active = active;
        }
        if (active != 0) {
            SimStatus status = accumulate_fault(fault, effects);

            if (status != SIM_OK) {
                return status;
            }
        }
    }
    return SIM_OK;
}

void fault_injection_apply_sensor(const FaultStepEffects *effects, SensorFrame *sensor)
{
    if (effects == 0 || sensor == 0) {
        return;
    }
    sensor->target_range_meas += effects->seeker_range_bias_m;
    sensor->target_los_unit_ecef_meas =
        vec3_add(sensor->target_los_unit_ecef_meas, effects->seeker_los_unit_bias);
    if (vec3_norm(sensor->target_los_unit_ecef_meas) > 0.0) {
        Vec3 normalized;

        if (vec3_normalize(sensor->target_los_unit_ecef_meas, &normalized) == SIM_OK) {
            sensor->target_los_unit_ecef_meas = normalized;
        }
    }
    sensor->target_los_rate_ecef_meas =
        vec3_add(sensor->target_los_rate_ecef_meas, effects->seeker_los_rate_bias_radps);
    sensor->target_closing_velocity_meas += effects->seeker_closing_velocity_bias_mps;
    sensor->missile_gyro_b_meas =
        vec3_add(sensor->missile_gyro_b_meas, effects->gyro_bias_b_radps);
    sensor->missile_accel_ecef_meas =
        vec3_add(sensor->missile_accel_ecef_meas, effects->accel_bias_ecef_mps2);
    sensor->missile_vel_ecef_meas =
        vec3_add(sensor->missile_vel_ecef_meas, effects->speed_bias_ecef_mps);
    sensor->sensor_valid_flags &= ~effects->sensor_valid_clear_mask;
    sensor->sensor_fault_flags |= effects->sensor_fault_set_mask;
}

void fault_injection_apply_actuators(
    const FaultStepEffects *effects,
    ActuatorState actuators[3],
    double commands[3])
{
    size_t index;

    if (effects == 0 || actuators == 0 || commands == 0) {
        return;
    }
    for (index = 0u; index < 3u; ++index) {
        commands[index] *= effects->actuator_command_scale[index];
        if (effects->actuator_stuck[index] != 0) {
            actuators[index].fault_flags |= ACTUATOR_FAULT_STUCK;
        } else {
            actuators[index].fault_flags &= ~ACTUATOR_FAULT_STUCK;
        }
    }
}
