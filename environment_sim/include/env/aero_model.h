/** @file aero_model.h
 *  @brief 基础气动力和控制力矩模型。
 */
#ifndef ENV_AERO_MODEL_H
#define ENV_AERO_MODEL_H

#include "common/status.h"
#include "common/vec3.h"

/** @brief 气动模型配置。 */
typedef struct AeroModel {
    int enabled;
    double reference_area_m2;
    double reference_length_m;
    double drag_coefficient;
    double control_force_coefficient;
    double control_moment_coefficient;
} AeroModel;

/** @brief 根据机体系相对气流和舵偏计算气动力、力矩。 */
SimStatus aero_model_evaluate(
    const AeroModel *model,
    double density_kgpm3,
    Vec3 velocity_air_b_mps,
    double pitch_actuator_rad,
    double yaw_actuator_rad,
    Vec3 *force_b_n,
    Vec3 *moment_b_nm);

#endif
