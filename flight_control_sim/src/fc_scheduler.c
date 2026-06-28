/** @file fc_scheduler.c
 *  @brief 飞控周期任务调度视图实现。
 */
#include "fc/fc_scheduler.h"

#include <math.h>
#include <stddef.h>

SimStatus fc_scheduler_init(FcScheduler *scheduler, double base_rate_hz)
{
    if (scheduler == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!isfinite(base_rate_hz) || base_rate_hz <= 0.0) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    scheduler->base_rate_hz = base_rate_hz;
    scheduler->tick = 0u;
    scheduler->task_count = 0u;
    return SIM_OK;
}

SimStatus fc_scheduler_add_task(FcScheduler *scheduler, const char *name, uint32_t period_ticks)
{
    FcTask *task;

    if (scheduler == 0 || name == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (period_ticks == 0u) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (scheduler->task_count >= FC_SCHEDULER_MAX_TASKS) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    task = &scheduler->tasks[scheduler->task_count++];
    task->name = name;
    task->period_ticks = period_ticks;
    task->last_run_tick = 0u;
    return SIM_OK;
}

int fc_scheduler_task_due(const FcScheduler *scheduler, uint32_t task_index)
{
    const FcTask *task;

    if (scheduler == 0 || task_index >= scheduler->task_count) {
        return 0;
    }
    task = &scheduler->tasks[task_index];
    return task->period_ticks != 0u && (scheduler->tick % task->period_ticks) == 0u;
}

void fc_scheduler_advance(FcScheduler *scheduler)
{
    if (scheduler != 0) {
        ++scheduler->tick;
    }
}
