/** @file target_model.h
 *  @brief 目标 ECEF 真值运动和脚本机动模型。
 */
#ifndef ENV_TARGET_MODEL_H
#define ENV_TARGET_MODEL_H

#include "common/config.h"
#include "common/status.h"
#include "common/vec3.h"

#include <stddef.h>

/** @brief 单个场景允许的最大目标机动段数量。 */
#define TARGET_MODEL_MAX_MANEUVERS 32u

/** @brief 目标运动模型类型。 */
typedef enum TargetModelType {
    TARGET_MODEL_CONSTANT_VELOCITY = 0,
    TARGET_MODEL_SCRIPTED_ACCELERATION = 1
} TargetModelType;

/** @brief 一个按仿真时间激活的 ECEF 常加速度机动段。 */
typedef struct TargetManeuver {
    /** @brief 机动开始时间，单位 s。 */
    double start_time_s;
    /** @brief 机动持续时间，单位 s。 */
    double duration_s;
    /** @brief 机动段 ECEF 加速度，单位 m/s^2。 */
    Vec3 acceleration_ecef_mps2;
} TargetManeuver;

/** @brief 目标运动模型只读配置。 */
typedef struct TargetModel {
    /** @brief 运动模型类型。 */
    TargetModelType type;
    /** @brief 有效机动段数量。 */
    size_t maneuver_count;
    /** @brief 固定容量机动段；重叠段的加速度按向量求和。 */
    TargetManeuver maneuvers[TARGET_MODEL_MAX_MANEUVERS];
} TargetModel;

/** @brief 从 scenario 的 `target` 节加载并严格校验目标模型。 */
SimStatus target_model_load_config(const ConfigTree *scenario, TargetModel *model);

/** @brief 校验目标模型的时间和有限值约束。 */
SimStatus target_model_validate(const TargetModel *model);

/** @brief 在一个固定仿真步内推进目标 ECEF 位置和速度。
 *
 *  函数会在机动开始/结束边界处分段积分，因此即使边界位于步内也不会把整个步错误地
 *  归入某一侧。仿真步开始时间为 @p sim_time_s，状态原地更新。
 */
SimStatus target_model_step(
    const TargetModel *model,
    double sim_time_s,
    double dt_s,
    Vec3 *position_ecef_m,
    Vec3 *velocity_ecef_mps);

/** @brief 返回用于 manifest 的稳定模型名称。 */
const char *target_model_type_to_string(TargetModelType type);

#endif
