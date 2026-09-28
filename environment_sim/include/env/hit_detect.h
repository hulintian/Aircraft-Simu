/** @file hit_detect.h
 *  @brief 步间连续相对运动最近点和命中判定。
 */
#ifndef ENV_HIT_DETECT_H
#define ENV_HIT_DETECT_H

#include "common/status.h"
#include "common/vec3.h"

/** @brief 一个仿真步内的线性相对轨迹判定结果。 */
typedef struct HitDetectResult {
    /** @brief 步内最近距离，单位 m。 */
    double minimum_range_m;
    /** @brief 最近点相对步长比例，范围 [0,1]。 */
    double closest_fraction;
    /** @brief 非零表示连续线段进入命中半径。 */
    int hit;
} HitDetectResult;

/** @brief 对导弹与目标在一个仿真步内的相对线段执行连续最近点判定。 */
SimStatus hit_detect_segment(
    Vec3 missile_start_ecef_m,
    Vec3 missile_end_ecef_m,
    Vec3 target_start_ecef_m,
    Vec3 target_end_ecef_m,
    double hit_radius_m,
    HitDetectResult *result);

#endif
