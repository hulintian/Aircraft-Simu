/** @file mass_model.h
 *  @brief 干质量、推进剂质量和质量流量模型。
 */
#ifndef ENV_MASS_MODEL_H
#define ENV_MASS_MODEL_H

#include "common/status.h"
#include "common/matrix3.h"
#include "common/vec3.h"

/** @brief 质量模型参数。 */
typedef struct MassModel {
    double mass_kg;
    /** @brief 不可消耗干质量，单位千克。 */
    double dry_mass_kg;
    /** @brief 当前推进剂质量，单位千克。 */
    double propellant_mass_kg;
    /** @brief 初始满装推进剂质量，单位千克。 */
    double initial_propellant_mass_kg;
    /** @brief 非零表示已配置质心和完整惯量张量演化。 */
    int properties_enabled;
    /** @brief 干体质心，机体系，单位米。 */
    Vec3 dry_center_of_mass_b_m;
    /** @brief 满装推进剂质心，机体系，单位米。 */
    Vec3 propellant_center_of_mass_full_b_m;
    /** @brief 空箱时推进剂等效质心，机体系，单位米。 */
    Vec3 propellant_center_of_mass_empty_b_m;
    /** @brief 干体绕自身质心的惯量张量，机体系，单位 kg*m^2。 */
    Matrix3 dry_inertia_centroid_b_kgm2;
    /** @brief 满装推进剂绕自身质心的惯量张量，机体系，单位 kg*m^2。 */
    Matrix3 propellant_inertia_full_centroid_b_kgm2;
    /** @brief 当前合成质心，机体系，单位米。 */
    Vec3 center_of_mass_b_m;
    /** @brief 当前绕合成质心的惯量张量，机体系，单位 kg*m^2。 */
    Matrix3 inertia_b_kgm2;
} MassModel;

/** @brief 初始化干质量和推进剂质量。 */
SimStatus mass_model_init(MassModel *model, double dry_mass_kg, double propellant_mass_kg);
/** @brief 配置干体/推进剂质心与完整惯量张量演化。 */
SimStatus mass_model_configure_properties(
    MassModel *model,
    Vec3 dry_center_of_mass_b_m,
    Vec3 propellant_center_of_mass_full_b_m,
    Vec3 propellant_center_of_mass_empty_b_m,
    Matrix3 dry_inertia_centroid_b_kgm2,
    Matrix3 propellant_inertia_full_centroid_b_kgm2);
/** @brief 按质量流量消耗推进剂并更新总质量。 */
SimStatus mass_model_step(MassModel *model, double mass_flow_kgps, double dt);
/** @brief 返回当前合成质心和绕该质心的完整惯量张量。 */
SimStatus mass_model_get_properties(
    const MassModel *model,
    Vec3 *center_of_mass_b_m,
    Matrix3 *inertia_b_kgm2);

#endif
