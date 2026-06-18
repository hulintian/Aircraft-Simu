/** @file quaternion.h
 *  @brief 姿态四元数数据结构与运算。
 *
 *  四元数采用标量在前格式。姿态积分使用机体系角速度，并在每次积分后
 *  归一化，防止长期运行产生单位模漂移。
 */
#ifndef COMMON_QUATERNION_H
#define COMMON_QUATERNION_H

#include "common/matrix3.h"
#include "common/status.h"
#include "common/vec3.h"

/** @brief 标量在前的四元数表示。 */
typedef struct Quat {
    double w;
    double x;
    double y;
    double z;
} Quat;

/** @brief 返回单位四元数。 */
Quat quat_identity(void);
/** @brief 判断四元数所有分量是否均为有限数。 */
int quat_isfinite(Quat quat);
/** @brief 归一化四元数。 */
SimStatus quat_normalize(Quat quat, Quat *out);
/** @brief 计算 Hamilton 四元数乘积。 */
Quat quat_multiply(Quat left, Quat right);
/** @brief 将单位四元数转换为方向余弦矩阵。 */
SimStatus quat_to_dcm(Quat quat, Matrix3 *out);
/** @brief 使用机体系角速度和一阶指数增量推进姿态。 */
SimStatus quat_integrate(Quat quat, Vec3 omega_body_radps, double dt, Quat *out);

#endif
