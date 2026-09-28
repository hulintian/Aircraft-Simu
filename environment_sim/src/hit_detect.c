/** @file hit_detect.c
 *  @brief 步间连续相对运动最近点和命中判定实现。
 */
#include "env/hit_detect.h"

#include <math.h>

SimStatus hit_detect_segment(
    Vec3 missile_start_ecef_m,
    Vec3 missile_end_ecef_m,
    Vec3 target_start_ecef_m,
    Vec3 target_end_ecef_m,
    double hit_radius_m,
    HitDetectResult *result)
{
    const Vec3 relative_start = vec3_sub(target_start_ecef_m, missile_start_ecef_m);
    const Vec3 relative_end = vec3_sub(target_end_ecef_m, missile_end_ecef_m);
    const Vec3 relative_delta = vec3_sub(relative_end, relative_start);
    const double delta_norm_squared = vec3_dot(relative_delta, relative_delta);
    double fraction = 0.0;

    if (result == 0 || !vec3_isfinite(missile_start_ecef_m) ||
        !vec3_isfinite(missile_end_ecef_m) || !vec3_isfinite(target_start_ecef_m) ||
        !vec3_isfinite(target_end_ecef_m) || !isfinite(hit_radius_m) || hit_radius_m < 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (delta_norm_squared > 0.0) {
        fraction = -vec3_dot(relative_start, relative_delta) / delta_norm_squared;
        if (fraction < 0.0) {
            fraction = 0.0;
        } else if (fraction > 1.0) {
            fraction = 1.0;
        }
    }
    result->minimum_range_m = vec3_norm(
        vec3_add(relative_start, vec3_scale(relative_delta, fraction)));
    result->closest_fraction = fraction;
    result->hit = result->minimum_range_m <= hit_radius_m;
    return isfinite(result->minimum_range_m) ? SIM_OK : SIM_ERR_NUMERIC;
}
