/** @file fc_scheduler.h
 *  @brief 飞控任务调度视图。
 */
#ifndef FC_FC_SCHEDULER_H
#define FC_FC_SCHEDULER_H

#include <stdint.h>

/** @brief 单个周期任务的调度信息。 */
typedef struct FcTask {
    /** @brief 任务名称。 */
    const char *name;
    /** @brief 周期，单位 tick。 */
    uint32_t period_ticks;
    /** @brief 上次执行的 tick。 */
    uint32_t last_run_tick;
} FcTask;

#endif
