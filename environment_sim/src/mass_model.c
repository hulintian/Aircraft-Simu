/** @file mass_model.c
 *  @brief 干质量和推进剂消耗模型实现。
 */
#include "env/mass_model.h"

#include <math.h>
#include <string.h>

/** @brief 检查矩阵是否为有限对称正定张量。 */
static int inertia_is_symmetric_positive_definite(Matrix3 inertia)
{
    const double symmetry_tolerance = 1.0e-12;
    const double minor_1 = inertia.m[0][0];
    const double minor_2 =
        (inertia.m[0][0] * inertia.m[1][1]) -
        (inertia.m[0][1] * inertia.m[1][0]);
    const double determinant =
        inertia.m[0][0] *
            ((inertia.m[1][1] * inertia.m[2][2]) - (inertia.m[1][2] * inertia.m[2][1])) -
        inertia.m[0][1] *
            ((inertia.m[1][0] * inertia.m[2][2]) - (inertia.m[1][2] * inertia.m[2][0])) +
        inertia.m[0][2] *
            ((inertia.m[1][0] * inertia.m[2][1]) - (inertia.m[1][1] * inertia.m[2][0]));
    size_t row;
    size_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            if (!isfinite(inertia.m[row][column])) {
                return 0;
            }
        }
    }
    return fabs(inertia.m[0][1] - inertia.m[1][0]) <= symmetry_tolerance &&
        fabs(inertia.m[0][2] - inertia.m[2][0]) <= symmetry_tolerance &&
        fabs(inertia.m[1][2] - inertia.m[2][1]) <= symmetry_tolerance &&
        minor_1 > 0.0 && minor_2 > 0.0 && determinant > 0.0;
}

/** @brief 将点质量平移到组合质心的平行轴项累加到惯量。 */
static void add_parallel_axis(Matrix3 *inertia, double mass_kg, Vec3 offset_b_m)
{
    const double distance_squared = vec3_dot(offset_b_m, offset_b_m);
    size_t row;
    size_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            const double component[3] = { offset_b_m.x, offset_b_m.y, offset_b_m.z };
            const double identity = row == column ? 1.0 : 0.0;

            inertia->m[row][column] +=
                mass_kg * ((distance_squared * identity) - (component[row] * component[column]));
        }
    }
}

/** @brief 根据当前推进剂质量重新组合质心和惯量。 */
static SimStatus update_mass_properties(MassModel *model)
{
    double fraction;
    Vec3 propellant_center;
    Matrix3 propellant_inertia;
    Matrix3 combined;
    size_t row;
    size_t column;

    if (model == 0 || model->mass_kg <= 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (model->properties_enabled == 0) {
        return SIM_OK;
    }
    fraction = model->initial_propellant_mass_kg > 0.0 ?
        model->propellant_mass_kg / model->initial_propellant_mass_kg :
        0.0;
    if (fraction < 0.0) {
        fraction = 0.0;
    } else if (fraction > 1.0) {
        fraction = 1.0;
    }
    propellant_center = vec3_add(
        model->propellant_center_of_mass_empty_b_m,
        vec3_scale(
            vec3_sub(
                model->propellant_center_of_mass_full_b_m,
                model->propellant_center_of_mass_empty_b_m),
            fraction));
    model->center_of_mass_b_m = vec3_scale(
        vec3_add(
            vec3_scale(model->dry_center_of_mass_b_m, model->dry_mass_kg),
            vec3_scale(propellant_center, model->propellant_mass_kg)),
        1.0 / model->mass_kg);
    propellant_inertia = matrix3_zero();
    combined = model->dry_inertia_centroid_b_kgm2;
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            propellant_inertia.m[row][column] =
                model->propellant_inertia_full_centroid_b_kgm2.m[row][column] * fraction;
            combined.m[row][column] += propellant_inertia.m[row][column];
        }
    }
    add_parallel_axis(
        &combined,
        model->dry_mass_kg,
        vec3_sub(model->dry_center_of_mass_b_m, model->center_of_mass_b_m));
    if (model->propellant_mass_kg > 0.0) {
        add_parallel_axis(
            &combined,
            model->propellant_mass_kg,
            vec3_sub(propellant_center, model->center_of_mass_b_m));
    }
    if (!vec3_isfinite(model->center_of_mass_b_m) ||
        !inertia_is_symmetric_positive_definite(combined)) {
        return SIM_ERR_NUMERIC;
    }
    model->inertia_b_kgm2 = combined;
    return SIM_OK;
}

/** @brief 初始化质量组成并计算总质量。 */
SimStatus mass_model_init(MassModel *model, double dry_mass_kg, double propellant_mass_kg)
{
    if (model == 0 || !isfinite(dry_mass_kg) || !isfinite(propellant_mass_kg) ||
        dry_mass_kg <= 0.0 || propellant_mass_kg < 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(model, 0, sizeof(*model));
    model->dry_mass_kg = dry_mass_kg;
    model->propellant_mass_kg = propellant_mass_kg;
    model->initial_propellant_mass_kg = propellant_mass_kg;
    model->mass_kg = dry_mass_kg + propellant_mass_kg;
    return SIM_OK;
}

SimStatus mass_model_configure_properties(
    MassModel *model,
    Vec3 dry_center_of_mass_b_m,
    Vec3 propellant_center_of_mass_full_b_m,
    Vec3 propellant_center_of_mass_empty_b_m,
    Matrix3 dry_inertia_centroid_b_kgm2,
    Matrix3 propellant_inertia_full_centroid_b_kgm2)
{
    if (model == 0 || !vec3_isfinite(dry_center_of_mass_b_m) ||
        !vec3_isfinite(propellant_center_of_mass_full_b_m) ||
        !vec3_isfinite(propellant_center_of_mass_empty_b_m) ||
        !inertia_is_symmetric_positive_definite(dry_inertia_centroid_b_kgm2) ||
        (model->initial_propellant_mass_kg > 0.0 &&
            !inertia_is_symmetric_positive_definite(propellant_inertia_full_centroid_b_kgm2))) {
        return SIM_ERR_INVALID_ARG;
    }
    model->dry_center_of_mass_b_m = dry_center_of_mass_b_m;
    model->propellant_center_of_mass_full_b_m = propellant_center_of_mass_full_b_m;
    model->propellant_center_of_mass_empty_b_m = propellant_center_of_mass_empty_b_m;
    model->dry_inertia_centroid_b_kgm2 = dry_inertia_centroid_b_kgm2;
    model->propellant_inertia_full_centroid_b_kgm2 = propellant_inertia_full_centroid_b_kgm2;
    model->properties_enabled = 1;
    return update_mass_properties(model);
}

/** @brief 消耗推进剂，且保证总质量不会低于干质量。 */
SimStatus mass_model_step(MassModel *model, double mass_flow_kgps, double dt)
{
    double consumed;

    if (model == 0 || !isfinite(mass_flow_kgps) || !isfinite(dt) ||
        mass_flow_kgps < 0.0 || dt < 0.0 ||
        model->dry_mass_kg <= 0.0 || model->propellant_mass_kg < 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    consumed = mass_flow_kgps * dt;
    if (consumed > model->propellant_mass_kg) {
        consumed = model->propellant_mass_kg;
    }
    model->propellant_mass_kg -= consumed;
    model->mass_kg = model->dry_mass_kg + model->propellant_mass_kg;
    return update_mass_properties(model);
}

SimStatus mass_model_get_properties(
    const MassModel *model,
    Vec3 *center_of_mass_b_m,
    Matrix3 *inertia_b_kgm2)
{
    if (model == 0 || center_of_mass_b_m == 0 || inertia_b_kgm2 == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (model->properties_enabled == 0 || !vec3_isfinite(model->center_of_mass_b_m) ||
        !inertia_is_symmetric_positive_definite(model->inertia_b_kgm2)) {
        return SIM_ERR_CONFIG;
    }
    *center_of_mass_b_m = model->center_of_mass_b_m;
    *inertia_b_kgm2 = model->inertia_b_kgm2;
    return SIM_OK;
}
