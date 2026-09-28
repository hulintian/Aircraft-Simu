/** @file wind_model.c
 *  @brief 可复现风切变、阵风和高斯-马尔可夫湍流实现。
 */
#include "env/wind_model.h"

#include "common/math_constants.h"

#include <math.h>
#include <string.h>

/** @brief 校验风场配置。 */
static SimStatus wind_model_config_validate(const WindModelConfig *config)
{
    if (config == 0 || !vec3_isfinite(config->base_velocity_ecef_mps) ||
        !vec3_isfinite(config->shear_ecef_per_m) ||
        !isfinite(config->reference_height_m) ||
        !vec3_isfinite(config->gust_amplitude_ecef_mps) ||
        !isfinite(config->gust_frequency_hz) || config->gust_frequency_hz < 0.0 ||
        !vec3_isfinite(config->turbulence_sigma_ecef_mps) ||
        config->turbulence_sigma_ecef_mps.x < 0.0 ||
        config->turbulence_sigma_ecef_mps.y < 0.0 ||
        config->turbulence_sigma_ecef_mps.z < 0.0 ||
        !isfinite(config->turbulence_time_constant_s) ||
        config->turbulence_time_constant_s <= 0.0) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    return SIM_OK;
}

SimStatus wind_model_init(WindModel *model, const WindModelConfig *config, uint64_t random_seed)
{
    SimStatus status;

    if (model == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = wind_model_config_validate(config);
    if (status != SIM_OK) {
        return status;
    }
    (void)memset(model, 0, sizeof(*model));
    model->config = *config;
    sim_random_seed(&model->random, random_seed);
    return SIM_OK;
}

SimStatus wind_model_step(
    WindModel *model,
    double sim_time_s,
    double height_m,
    double dt_s,
    Vec3 *wind_velocity_ecef_mps)
{
    double decay;
    double innovation_scale;
    double gust_phase;
    Vec3 shear;
    Vec3 gust;

    if (model == 0 || wind_velocity_ecef_mps == 0 ||
        !isfinite(sim_time_s) || !isfinite(height_m) || !isfinite(dt_s) || dt_s <= 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (wind_model_config_validate(&model->config) != SIM_OK) {
        return SIM_ERR_CONFIG;
    }
    if (model->config.enabled == 0) {
        *wind_velocity_ecef_mps = vec3_make(0.0, 0.0, 0.0);
        return SIM_OK;
    }
    decay = exp(-dt_s / model->config.turbulence_time_constant_s);
    innovation_scale = sqrt(1.0 - (decay * decay));
    model->turbulence_state_ecef_mps.x =
        decay * model->turbulence_state_ecef_mps.x +
        innovation_scale * sim_random_normal(
            &model->random,
            0.0,
            model->config.turbulence_sigma_ecef_mps.x);
    model->turbulence_state_ecef_mps.y =
        decay * model->turbulence_state_ecef_mps.y +
        innovation_scale * sim_random_normal(
            &model->random,
            0.0,
            model->config.turbulence_sigma_ecef_mps.y);
    model->turbulence_state_ecef_mps.z =
        decay * model->turbulence_state_ecef_mps.z +
        innovation_scale * sim_random_normal(
            &model->random,
            0.0,
            model->config.turbulence_sigma_ecef_mps.z);
    shear = vec3_scale(
        model->config.shear_ecef_per_m,
        height_m - model->config.reference_height_m);
    gust_phase = sin(SIM_TWO_PI * model->config.gust_frequency_hz * sim_time_s);
    gust = vec3_scale(model->config.gust_amplitude_ecef_mps, gust_phase);
    *wind_velocity_ecef_mps = vec3_add(
        model->config.base_velocity_ecef_mps,
        vec3_add(shear, vec3_add(gust, model->turbulence_state_ecef_mps)));
    return vec3_isfinite(*wind_velocity_ecef_mps) ? SIM_OK : SIM_ERR_NUMERIC;
}
