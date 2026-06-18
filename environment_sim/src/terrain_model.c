/** @file terrain_model.c
 *  @brief 地形查询、碰撞和视线遮挡实现。
 */
#include "env/terrain_model.h"

#include <float.h>
#include <math.h>

#define TERRAIN_WARNING_MISSING_TILE UINT32_C(0x00000001)

/** @brief 绑定只读瓦片集合并配置缺失瓦片策略。 */
SimStatus terrain_model_init(
    TerrainModel *terrain,
    const TerrainTile *tiles,
    size_t tile_count,
    TerrainMissingPolicy missing_policy,
    double flat_fill_height_m)
{
    if (terrain == 0 || (tiles == 0 && tile_count != 0u)) {
        return SIM_ERR_INVALID_ARG;
    }
    if (missing_policy < MAP_MISSING_ERROR || missing_policy > MAP_MISSING_NEAREST ||
        !isfinite(flat_fill_height_m)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    terrain->enabled = 1;
    terrain->tiles = tiles;
    terrain->tile_count = tile_count;
    terrain->missing_policy = missing_policy;
    terrain->flat_fill_height_m = flat_fill_height_m;
    terrain->warning_flags = 0u;
    return SIM_OK;
}

/** @brief 将标量限制在闭区间内。 */
static double clamp_value(double value, double minimum, double maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/** @brief 查找覆盖查询点的瓦片并返回插值高程。 */
SimStatus terrain_get_height(
    TerrainModel *terrain,
    double lat_rad,
    double lon_rad,
    double *height_m)
{
    size_t index;

    if (terrain == 0 || height_m == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!isfinite(lat_rad) || !isfinite(lon_rad)) {
        return SIM_ERR_NUMERIC;
    }
    if (terrain->enabled == 0) {
        *height_m = terrain->flat_fill_height_m;
        return SIM_OK;
    }
    for (index = 0u; index < terrain->tile_count; ++index) {
        if (map_tile_contains(&terrain->tiles[index], lat_rad, lon_rad)) {
            return map_tile_get_height(&terrain->tiles[index], lat_rad, lon_rad, height_m);
        }
    }

    terrain->warning_flags |= TERRAIN_WARNING_MISSING_TILE;
    if (terrain->missing_policy == MAP_MISSING_FLAT_FILL) {
        *height_m = terrain->flat_fill_height_m;
        return SIM_OK;
    }
    if (terrain->missing_policy == MAP_MISSING_NEAREST && terrain->tile_count > 0u) {
        const TerrainTile *nearest = 0;
        double nearest_distance = DBL_MAX;

        for (index = 0u; index < terrain->tile_count; ++index) {
            const TerrainTile *tile = &terrain->tiles[index];
            const double sample_lat = clamp_value(lat_rad, tile->header.lat_min, tile->header.lat_max);
            const double sample_lon = clamp_value(lon_rad, tile->header.lon_min, tile->header.lon_max);
            const double dlat = lat_rad - sample_lat;
            const double dlon = lon_rad - sample_lon;
            const double distance = (dlat * dlat) + (dlon * dlon);

            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest = tile;
            }
        }
        if (nearest != 0) {
            return map_tile_get_height(
                nearest,
                clamp_value(lat_rad, nearest->header.lat_min, nearest->header.lat_max),
                clamp_value(lon_rad, nearest->header.lon_min, nearest->header.lon_max),
                height_m);
        }
    }
    return SIM_ERR_OUT_OF_RANGE;
}

/** @brief 计算椭球高与地形高之差。 */
SimStatus terrain_get_agl(
    TerrainModel *terrain,
    const LlaCoord *position,
    double *agl_m)
{
    double terrain_height;
    SimStatus status;

    if (position == 0 || agl_m == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = terrain_get_height(terrain, position->lat_rad, position->lon_rad, &terrain_height);
    if (status != SIM_OK) {
        return status;
    }
    *agl_m = position->height_m - terrain_height;
    return isfinite(*agl_m) ? SIM_OK : SIM_ERR_NUMERIC;
}

/** @brief 以 AGL 小于等于零作为地表碰撞判据。 */
SimStatus terrain_is_surface_collision(
    TerrainModel *terrain,
    const LlaCoord *position,
    int *collision)
{
    double agl;
    SimStatus status;

    if (collision == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = terrain_get_agl(terrain, position, &agl);
    if (status == SIM_OK) {
        *collision = agl <= 0.0;
    }
    return status;
}

/** @brief 沿 ECEF 线段内部等距采样并判断是否穿入地形。
 *
 *  @param interior_sample_count 不包含起点和终点的内部采样数量。
 *  该函数不分配内存，适合在导引头更新周期内调用。
 */
SimStatus terrain_line_of_sight_occluded(
    TerrainModel *terrain,
    const EarthModel *earth,
    Vec3 start_ecef_m,
    Vec3 end_ecef_m,
    size_t interior_sample_count,
    int *occluded)
{
    size_t index;

    if (terrain == 0 || earth == 0 || occluded == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!vec3_isfinite(start_ecef_m) || !vec3_isfinite(end_ecef_m)) {
        return SIM_ERR_NUMERIC;
    }
    *occluded = 0;
    for (index = 1u; index <= interior_sample_count; ++index) {
        const double fraction = (double)index / (double)(interior_sample_count + 1u);
        EcefCoord sample_ecef;
        LlaCoord sample_lla;
        double terrain_height;
        SimStatus status;

        sample_ecef.position_m = vec3_add(
            start_ecef_m,
            vec3_scale(vec3_sub(end_ecef_m, start_ecef_m), fraction));
        status = geo_ecef_to_lla(earth, &sample_ecef, &sample_lla);
        if (status != SIM_OK) {
            return status;
        }
        status = terrain_get_height(
            terrain,
            sample_lla.lat_rad,
            sample_lla.lon_rad,
            &terrain_height);
        if (status != SIM_OK) {
            return status;
        }
        if (sample_lla.height_m <= terrain_height) {
            *occluded = 1;
            return SIM_OK;
        }
    }
    return SIM_OK;
}
