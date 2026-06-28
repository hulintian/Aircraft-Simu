/** @file fc_scheduler.h
 *  @brief 飞控任务调度视图。
 *
 *  当前飞控进程由 LOCKSTEP 传感器帧驱动，因此调度器不主动睡眠或抢占。
 *  该模块保存配置中的基准频率和任务周期，用于校验任务表，并为后续
 *  多速率导航/控制任务扩展提供一致的 tick 判定接口。
 */
#ifndef FC_FC_SCHEDULER_H
#define FC_FC_SCHEDULER_H

#include "common/status.h"

#include <stdint.h>

/** @brief 飞控调度器最多保存的任务数量。 */
#define FC_SCHEDULER_MAX_TASKS 8u

/** @brief 单个周期任务的调度信息。 */
typedef struct FcTask {
    /** @brief 任务名称。 */
    const char *name;
    /** @brief 周期，单位 tick。 */
    uint32_t period_ticks;
    /** @brief 上次执行的 tick。 */
    uint32_t last_run_tick;
} FcTask;

/** @brief 飞控调度器状态。
 *
 *  @var base_rate_hz
 *  配置中的基准频率，单位 Hz，必须为正有限数。
 *  @var tick
 *  已处理传感器帧对应的离散调度 tick。
 */
typedef struct FcScheduler {
    double base_rate_hz;
    uint32_t tick;
    FcTask tasks[FC_SCHEDULER_MAX_TASKS];
    uint32_t task_count;
} FcScheduler;

/** @brief 初始化飞控调度器。 */
SimStatus fc_scheduler_init(FcScheduler *scheduler, double base_rate_hz);

/** @brief 注册一个周期任务。 */
SimStatus fc_scheduler_add_task(FcScheduler *scheduler, const char *name, uint32_t period_ticks);

/** @brief 判断指定任务在当前 tick 是否到期。 */
int fc_scheduler_task_due(const FcScheduler *scheduler, uint32_t task_index);

/** @brief 推进一个调度 tick。 */
void fc_scheduler_advance(FcScheduler *scheduler);

#endif
