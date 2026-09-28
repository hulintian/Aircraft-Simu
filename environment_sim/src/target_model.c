/** @file target_model.c
 *  @brief 目标 ECEF 真值运动和脚本机动模型实现。
 */
#include "env/target_model.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TARGET_BOUNDARY_CAPACITY ((2u * TARGET_MODEL_MAX_MANEUVERS) + 2u)

static Vec3 target_acceleration_at(const TargetModel *model, double time_s)
{
    Vec3 acceleration = vec3_make(0.0, 0.0, 0.0);
    size_t index;

    for (index = 0u; index < model->maneuver_count; ++index) {
        const TargetManeuver *maneuver = &model->maneuvers[index];
        const double end_time_s = maneuver->start_time_s + maneuver->duration_s;

        if (time_s >= maneuver->start_time_s && time_s < end_time_s) {
            acceleration = vec3_add(acceleration, maneuver->acceleration_ecef_mps2);
        }
    }
    return acceleration;
}

static void sort_boundaries(double *values, size_t count)
{
    size_t index;

    for (index = 1u; index < count; ++index) {
        const double value = values[index];
        size_t insertion = index;

        while (insertion > 0u && values[insertion - 1u] > value) {
            values[insertion] = values[insertion - 1u];
            --insertion;
        }
        values[insertion] = value;
    }
}

SimStatus target_model_validate(const TargetModel *model)
{
    size_t index;

    if (model == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (model->type != TARGET_MODEL_CONSTANT_VELOCITY &&
        model->type != TARGET_MODEL_SCRIPTED_ACCELERATION) {
        return SIM_ERR_CONFIG;
    }
    if (model->maneuver_count > TARGET_MODEL_MAX_MANEUVERS ||
        (model->type == TARGET_MODEL_CONSTANT_VELOCITY && model->maneuver_count != 0u) ||
        (model->type == TARGET_MODEL_SCRIPTED_ACCELERATION && model->maneuver_count == 0u)) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < model->maneuver_count; ++index) {
        const TargetManeuver *maneuver = &model->maneuvers[index];

        if (!isfinite(maneuver->start_time_s) || maneuver->start_time_s < 0.0 ||
            !isfinite(maneuver->duration_s) || maneuver->duration_s <= 0.0 ||
            !vec3_isfinite(maneuver->acceleration_ecef_mps2) ||
            !isfinite(maneuver->start_time_s + maneuver->duration_s)) {
            return SIM_ERR_OUT_OF_RANGE;
        }
    }
    return SIM_OK;
}

SimStatus target_model_load_config(const ConfigTree *scenario, TargetModel *model)
{
    char model_name[40];
    size_t maneuver_count = 0u;
    size_t index;
    SimStatus status;

    if (scenario == 0 || model == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(model, 0, sizeof(*model));
    status = config_get_string(scenario, "target.model", model_name, sizeof(model_name));
    if (status != SIM_OK) {
        return status;
    }
    if (strcmp(model_name, "CONSTANT_VELOCITY") == 0) {
        model->type = TARGET_MODEL_CONSTANT_VELOCITY;
        if (config_get_array_count(scenario, "target.maneuvers", &maneuver_count) == SIM_OK &&
            maneuver_count != 0u) {
            return SIM_ERR_CONFIG;
        }
    } else if (strcmp(model_name, "SCRIPTED") == 0 ||
        strcmp(model_name, "SCRIPTED_ACCELERATION") == 0) {
        model->type = TARGET_MODEL_SCRIPTED_ACCELERATION;
        status = config_get_array_count(scenario, "target.maneuvers", &maneuver_count);
        if (status != SIM_OK || maneuver_count == 0u) {
            return status == SIM_OK ? SIM_ERR_CONFIG : status;
        }
    } else {
        return SIM_ERR_CONFIG;
    }
    if (maneuver_count > TARGET_MODEL_MAX_MANEUVERS) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    model->maneuver_count = maneuver_count;
    for (index = 0u; index < maneuver_count; ++index) {
        char path[128];
        double acceleration[3];
        TargetManeuver *maneuver = &model->maneuvers[index];

        (void)snprintf(path, sizeof(path), "target.maneuvers[%zu].start_time_s", index);
        status = config_get_double(scenario, path, &maneuver->start_time_s);
        if (status == SIM_OK) {
            (void)snprintf(path, sizeof(path), "target.maneuvers[%zu].duration_s", index);
            status = config_get_double(scenario, path, &maneuver->duration_s);
        }
        if (status == SIM_OK) {
            (void)snprintf(
                path,
                sizeof(path),
                "target.maneuvers[%zu].acceleration_ecef_mps2",
                index);
            status = config_get_double_array(scenario, path, acceleration, 3u);
        }
        if (status != SIM_OK) {
            return status;
        }
        maneuver->acceleration_ecef_mps2 =
            vec3_make(acceleration[0], acceleration[1], acceleration[2]);
    }
    return target_model_validate(model);
}

SimStatus target_model_step(
    const TargetModel *model,
    double sim_time_s,
    double dt_s,
    Vec3 *position_ecef_m,
    Vec3 *velocity_ecef_mps)
{
    double boundaries[TARGET_BOUNDARY_CAPACITY];
    double step_end_time_s;
    size_t boundary_count = 0u;
    size_t index;
    SimStatus status;

    if (position_ecef_m == 0 || velocity_ecef_mps == 0 ||
        !isfinite(sim_time_s) || !isfinite(dt_s) || dt_s <= 0.0 ||
        !vec3_isfinite(*position_ecef_m) || !vec3_isfinite(*velocity_ecef_mps)) {
        return SIM_ERR_INVALID_ARG;
    }
    step_end_time_s = sim_time_s + dt_s;
    if (!isfinite(step_end_time_s) || step_end_time_s <= sim_time_s) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    status = target_model_validate(model);
    if (status != SIM_OK) {
        return status;
    }
    boundaries[boundary_count++] = sim_time_s;
    boundaries[boundary_count++] = step_end_time_s;
    for (index = 0u; index < model->maneuver_count; ++index) {
        const double start = model->maneuvers[index].start_time_s;
        const double end = start + model->maneuvers[index].duration_s;

        if (start > sim_time_s && start < step_end_time_s) {
            boundaries[boundary_count++] = start;
        }
        if (end > sim_time_s && end < step_end_time_s) {
            boundaries[boundary_count++] = end;
        }
    }
    sort_boundaries(boundaries, boundary_count);
    for (index = 0u; index + 1u < boundary_count; ++index) {
        const double interval_s = boundaries[index + 1u] - boundaries[index];
        const double midpoint_s = boundaries[index] + (0.5 * interval_s);
        const Vec3 acceleration = target_acceleration_at(model, midpoint_s);

        if (interval_s <= 0.0) {
            continue;
        }
        *position_ecef_m = vec3_add(
            *position_ecef_m,
            vec3_add(
                vec3_scale(*velocity_ecef_mps, interval_s),
                vec3_scale(acceleration, 0.5 * interval_s * interval_s)));
        *velocity_ecef_mps = vec3_add(
            *velocity_ecef_mps,
            vec3_scale(acceleration, interval_s));
    }
    return vec3_isfinite(*position_ecef_m) && vec3_isfinite(*velocity_ecef_mps) ?
        SIM_OK : SIM_ERR_NUMERIC;
}

const char *target_model_type_to_string(TargetModelType type)
{
    return type == TARGET_MODEL_SCRIPTED_ACCELERATION ?
        "SCRIPTED_ACCELERATION" : "CONSTANT_VELOCITY";
}
