/** @file environment_force_model.c
 *  @brief 环境力、力矩和重力统一汇总实现。
 */
#include "env/environment_force_model.h"

#include "common/matrix3.h"
#include "common/quaternion.h"

#include <math.h>
#include <string.h>

/** @brief 校验环境力模型全部配置项。 */
SimStatus environment_force_model_validate(const EnvironmentForceModel *model)
{
    Vec3 normalized_direction;

    if (model == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!vec3_isfinite(model->wind_velocity_ecef_mps) ||
        !isfinite(model->earth_rotation_rate_radps) ||
        model->earth_rotation_rate_radps < 0.0 ||
        !isfinite(model->gravity.gravitational_parameter) ||
        model->gravity.gravitational_parameter <= 0.0 ||
        !isfinite(model->atmosphere.maximum_model_height_m) ||
        model->atmosphere.maximum_model_height_m <= 0.0 ||
        !isfinite(model->propulsion.thrust_n) ||
        model->propulsion.thrust_n < 0.0 ||
        !isfinite(model->propulsion.mass_flow_kgps) ||
        model->propulsion.mass_flow_kgps < 0.0 ||
        !isfinite(model->propulsion.burn_time_s) ||
        model->propulsion.burn_time_s < 0.0 ||
        !isfinite(model->aerodynamics.reference_area_m2) ||
        model->aerodynamics.reference_area_m2 < 0.0 ||
        !isfinite(model->aerodynamics.reference_length_m) ||
        model->aerodynamics.reference_length_m < 0.0 ||
        !isfinite(model->aerodynamics.drag_coefficient) ||
        model->aerodynamics.drag_coefficient < 0.0 ||
        !isfinite(model->aerodynamics.control_force_coefficient) ||
        !isfinite(model->aerodynamics.control_moment_coefficient)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (model->propulsion.enabled != 0 &&
        vec3_normalize(model->propulsion.thrust_direction_b, &normalized_direction) != SIM_OK) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    return SIM_OK;
}

/** @brief 计算并组合虚拟控制力、气动力、推进力和重力。 */
SimStatus environment_force_model_evaluate(
    const EnvironmentForceModel *model,
    const EnvironmentForceInput *input,
    EnvironmentForceOutput *output)
{
    Matrix3 body_to_ecef;
    Matrix3 ecef_to_body;
    Vec3 air_velocity_ecef;
    Vec3 air_velocity_body;
    Vec3 propulsion_force;
    double requested_mass_flow;
    double burn_fraction = 1.0;
    SimStatus status;

    if (model == 0 || input == 0 || output == 0 || input->plant == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = environment_force_model_validate(model);
    if (status != SIM_OK) {
        return status;
    }
    if (!isfinite(input->height_m) ||
        !vec3_isfinite(input->virtual_acceleration_ecef_mps2) ||
        !isfinite(input->pitch_actuator_rad) ||
        !isfinite(input->yaw_actuator_rad) ||
        !isfinite(input->propellant_mass_kg) ||
        input->propellant_mass_kg < 0.0 ||
        !isfinite(input->dt_s) ||
        input->dt_s <= 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = missile_plant_validate(input->plant);
    if (status != SIM_OK) {
        return status;
    }

    memset(output, 0, sizeof(*output));
    status = quat_to_dcm(input->plant->q_bi, &body_to_ecef);
    if (status != SIM_OK) {
        return status;
    }
    ecef_to_body = matrix3_transpose(body_to_ecef);

    status = atmosphere_model_evaluate(
        &model->atmosphere,
        input->height_m,
        &output->atmosphere);
    if (status != SIM_OK) {
        return status;
    }
    air_velocity_ecef = vec3_sub(
        input->plant->vel_ecef,
        model->wind_velocity_ecef_mps);
    air_velocity_body = matrix3_multiply_vec3(ecef_to_body, air_velocity_ecef);
    status = aero_model_evaluate(
        &model->aerodynamics,
        output->atmosphere.density_kgpm3,
        air_velocity_body,
        input->pitch_actuator_rad,
        input->yaw_actuator_rad,
        &output->aerodynamic_force_b_n,
        &output->aerodynamic_moment_b_nm);
    if (status != SIM_OK) {
        return status;
    }

    status = propulsion_model_evaluate(
        &model->propulsion,
        input->plant->time,
        &propulsion_force,
        &requested_mass_flow);
    if (status != SIM_OK) {
        return status;
    }
    if (requested_mass_flow > 0.0) {
        const double requested_propellant = requested_mass_flow * input->dt_s;

        if (input->propellant_mass_kg <= 0.0) {
            burn_fraction = 0.0;
        } else if (requested_propellant > input->propellant_mass_kg) {
            burn_fraction = input->propellant_mass_kg / requested_propellant;
        }
    }
    output->propulsion_force_b_n = vec3_scale(propulsion_force, burn_fraction);
    output->mass_flow_kgps = requested_mass_flow * burn_fraction;

    output->virtual_force_b_n = matrix3_multiply_vec3(
        ecef_to_body,
        vec3_scale(
            input->virtual_acceleration_ecef_mps2,
            input->plant->mass));
    output->plant_input.force_b_n = vec3_add(
        output->virtual_force_b_n,
        vec3_add(
            output->aerodynamic_force_b_n,
            output->propulsion_force_b_n));
    output->plant_input.moment_b_nm = output->aerodynamic_moment_b_nm;
    status = gravity_model_acceleration(
        &model->gravity,
        input->plant->pos_ecef,
        &output->plant_input.gravity_ecef_mps2);
    if (status != SIM_OK) {
        return status;
    }
    output->plant_input.enable_earth_rotation = model->enable_earth_rotation;
    output->plant_input.earth_rotation_ecef_radps = vec3_make(
        0.0,
        0.0,
        model->earth_rotation_rate_radps);
    return SIM_OK;
}
