/** @file vec3.h
 *  @brief 三维向量基础运算。
 *
 *  仿真系统内部大量使用 ECEF、机体系、惯性系向量。该头文件提供最小
 *  依赖的三维向量类型与基础代数运算，作为公共数值基础库。
 */
#ifndef COMMON_VEC3_H
#define COMMON_VEC3_H

#include "common/status.h"

/** @brief 三维笛卡尔向量。
 *
 *  约定单位由使用场景决定。对于位置通常是米，对于角速度可能是弧度每秒。
 */
typedef struct Vec3 {
    double x;
    double y;
    double z;
} Vec3;

/** @brief 构造一个三维向量。 */
Vec3 vec3_make(double x, double y, double z);
/** @brief 向量加法。 */
Vec3 vec3_add(Vec3 a, Vec3 b);
/** @brief 向量减法。 */
Vec3 vec3_sub(Vec3 a, Vec3 b);
/** @brief 向量数乘。 */
Vec3 vec3_scale(Vec3 v, double scale);
/** @brief 向量点积。 */
double vec3_dot(Vec3 a, Vec3 b);
/** @brief 向量叉积。 */
Vec3 vec3_cross(Vec3 a, Vec3 b);
/** @brief 向量模长。 */
double vec3_norm(Vec3 v);
/** @brief 判断向量三个分量是否均为有限数。 */
int vec3_isfinite(Vec3 v);
/** @brief 将向量归一化。 */
SimStatus vec3_normalize(Vec3 v, Vec3 *out);

#endif
