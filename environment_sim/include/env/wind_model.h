/** @file wind_model.h
 *  @brief 可复现基础风、风切变、阵风和一阶湍流模型。
 */
#ifndef ENV_WIND_MODEL_H
#define ENV_WIND_MODEL_H

#include "common/random.h"
#include "common/status.h"
#include "common/vec3.h"

#include <stdint.h>

/** @brief 风场模型配置，所有速度均为 ECEF 坐标。 */
typedef struct WindModelConfig {
    int enabled;
    Vec3 base_velocity_ecef_mps;
    /** @brief 单位高度变化引起的风速变化，单位 (m/s)/m。 */
    Vec3 shear_ecef_per_m;
    double reference_height_m;
    Vec3 gust_amplitude_ecef_mps;
    double gust_frequency_hz;
    /** @brief 各轴稳态湍流标准差，单位 m/s。 */
    Vec3 turbulence_sigma_ecef_mps;
    /** @brief 一阶高斯-马尔可夫湍流相关时间，单位 s。 */
    double turbulence_time_constant_s;
} WindModelConfig;

/** @brief 单实例风场运行状态。 */
typedef struct WindModel {
    WindModelConfig config;
    SimRandom random;
    Vec3 turbulence_state_ecef_mps;
} WindModel;

/** @brief 初始化实例私有风场状态。 */
SimStatus wind_model_init(WindModel *model, const WindModelConfig *config, uint64_t random_seed);

/** @brief 推进风场并返回当前 ECEF 风速。 */
SimStatus wind_model_step(
    WindModel *model,
    double sim_time_s,
    double height_m,
    double dt_s,
    Vec3 *wind_velocity_ecef_mps);

#endif
