/** @file guidance_png.h
 *  @brief 比例导引法制导接口。
 *
 *  该模块根据目标视线、视线角速度和闭合速度生成加速度指令，是飞控制导
 *  核心算法的主要入口。
 */
#ifndef FC_GUIDANCE_PNG_H
#define FC_GUIDANCE_PNG_H

#include "common/status.h"
#include "common/vec3.h"

/** @brief PNG 制导参数。 */
typedef struct GuidancePngConfig {
    /** @brief 导引系数。 */
    double navigation_constant;
    /** @brief 最大加速度限幅。 */
    double max_accel_mps2;
    /** @brief 最大加速度变化率限幅。 */
    double max_accel_rate_mps3;
} GuidancePngConfig;

/** @brief PNG 制导输入量。 */
typedef struct GuidancePngInput {
    /** @brief 当前目标距离，单位米。 */
    double range_m;
    /** @brief 闭合速度，单位米每秒。 */
    double closing_velocity_mps;
    /** @brief 视线单位向量，ECEF 坐标系。 */
    Vec3 los_unit_ecef;
    /** @brief 视线角速度向量，ECEF 坐标系。 */
    Vec3 los_rate_ecef;
} GuidancePngInput;

/** @brief PNG 制导输出量。 */
typedef struct GuidancePngOutput {
    /** @brief 生成的加速度指令，ECEF 坐标系。 */
    Vec3 accel_cmd_ecef;
} GuidancePngOutput;

/** @brief 执行一次 PNG 制导更新。 */
SimStatus guidance_png_update(
    const GuidancePngConfig *cfg,
    const GuidancePngInput *in,
    GuidancePngOutput *out);

#endif
