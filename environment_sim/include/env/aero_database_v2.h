/** @file aero_database_v2.h
 *  @brief 六维规则网格气动系数表及多线性插值。
 */
#ifndef ENV_AERO_DATABASE_V2_H
#define ENV_AERO_DATABASE_V2_H

#include "common/status.h"
#include "env/aero_database.h"

#include <stddef.h>
#include <stdint.h>

#define AERO_DATABASE_V2_MAGIC UINT32_C(0x32524541)
#define AERO_DATABASE_V2_VERSION 2u
#define AERO_DATABASE_V2_HEADER_WIRE_SIZE 48u
#define AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE 48u
#define AERO_DATABASE_V2_AXIS_COUNT 6u

/** @brief 一个规则网格点上的六个无量纲气动系数。 */
typedef struct AeroCoefficientSet {
    double cx;
    double cy;
    double cz;
    double cl;
    double cm;
    double cn;
} AeroCoefficientSet;

/** @brief v2 规则网格的只读输入视图，轴顺序固定。 */
typedef struct AeroDatabaseV2Grid {
    const double *mach_axis;
    size_t mach_count;
    const double *alpha_axis_rad;
    size_t alpha_count;
    const double *beta_axis_rad;
    size_t beta_count;
    const double *height_axis_m;
    size_t height_count;
    const double *pitch_actuator_axis_rad;
    size_t pitch_actuator_count;
    const double *yaw_actuator_axis_rad;
    size_t yaw_actuator_count;
    /** @brief 最后一个轴变化最快的行主序系数数组。 */
    const AeroCoefficientSet *coefficients;
    size_t coefficient_count;
} AeroDatabaseV2Grid;

/** @brief v2 数据库状态；加载后拥有轴和系数内存。 */
typedef struct AeroDatabaseV2 {
    double *axes[AERO_DATABASE_V2_AXIS_COUNT];
    size_t dimensions[AERO_DATABASE_V2_AXIS_COUNT];
    AeroCoefficientSet *coefficients;
    size_t coefficient_count;
    AeroDatabaseExtrapolationPolicy extrapolation_policy;
    AeroCoefficientSet last_valid;
    int have_last_valid;
} AeroDatabaseV2;

/** @brief 深拷贝并校验一个六维规则网格。 */
SimStatus aero_database_v2_init(
    AeroDatabaseV2 *database,
    const AeroDatabaseV2Grid *grid,
    AeroDatabaseExtrapolationPolicy extrapolation_policy);

/** @brief 写出固定小端、带 CRC 的 v2 气动表。 */
SimStatus aero_database_v2_write_file(
    const char *path,
    const AeroDatabaseV2Grid *grid,
    AeroDatabaseExtrapolationPolicy extrapolation_policy);

/** @brief 加载并校验 v2 气动表。 */
SimStatus aero_database_v2_load_file(const char *path, AeroDatabaseV2 *database);

/** @brief 释放数据库拥有的所有轴和系数内存。 */
void aero_database_v2_unload(AeroDatabaseV2 *database);

/** @brief 对六维规则网格执行多线性插值。 */
SimStatus aero_database_v2_lookup(
    AeroDatabaseV2 *database,
    double mach,
    double alpha_rad,
    double beta_rad,
    double height_m,
    double pitch_actuator_rad,
    double yaw_actuator_rad,
    AeroCoefficientSet *out,
    uint32_t *flags);

#endif
