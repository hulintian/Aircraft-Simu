/** @file terrain_model.h
 *  @brief 地形高程、AGL、碰撞和视线遮挡查询。
 */
#ifndef ENV_TERRAIN_MODEL_H
#define ENV_TERRAIN_MODEL_H

#include "common/status.h"
#include "common/vec3.h"
#include "env/earth_model.h"
#include "env/geo_coordinate.h"
#include "env/map_tile.h"

#include <stddef.h>
#include <stdint.h>

/** @brief 地形瓦片缺失时的处理策略。 */
typedef enum TerrainMissingPolicy {
    MAP_MISSING_ERROR = 0,
    MAP_MISSING_FLAT_FILL = 1,
    MAP_MISSING_NEAREST = 2
} TerrainMissingPolicy;

/** @brief 地形模型运行状态。 */
typedef struct TerrainModel {
    /** @brief 是否启用瓦片查询；关闭时返回平坦填充值。 */
    int enabled;
    /** @brief 只读瓦片数组，所有权属于调用方。 */
    const TerrainTile *tiles;
    /** @brief 瓦片数组元素数量。 */
    size_t tile_count;
    /** @brief 查询点不在任何瓦片内时的处理策略。 */
    TerrainMissingPolicy missing_policy;
    /** @brief 平坦填充策略使用的高程，单位米。 */
    double flat_fill_height_m;
    /** @brief 查询过程累计的非致命告警位。 */
    uint32_t warning_flags;
} TerrainModel;

/** @brief 初始化不拥有瓦片内存的地形查询模型。 */
SimStatus terrain_model_init(
    TerrainModel *terrain,
    const TerrainTile *tiles,
    size_t tile_count,
    TerrainMissingPolicy missing_policy,
    double flat_fill_height_m);
/** @brief 查询指定经纬度的地形高程。 */
SimStatus terrain_get_height(
    TerrainModel *terrain,
    double lat_rad,
    double lon_rad,
    double *height_m);
/** @brief 计算大地高度相对地形的 AGL。 */
SimStatus terrain_get_agl(
    TerrainModel *terrain,
    const LlaCoord *position,
    double *agl_m);
/** @brief 判断位置是否接触或进入地表。 */
SimStatus terrain_is_surface_collision(
    TerrainModel *terrain,
    const LlaCoord *position,
    int *collision);
/** @brief 沿 ECEF 线段采样判断地形遮挡。 */
SimStatus terrain_line_of_sight_occluded(
    TerrainModel *terrain,
    const EarthModel *earth,
    Vec3 start_ecef_m,
    Vec3 end_ecef_m,
    size_t interior_sample_count,
    int *occluded);

#endif
