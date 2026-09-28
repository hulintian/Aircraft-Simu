/** @file realtime_pacer.h
 *  @brief 基于单调墙钟的绝对截止时间实时节拍器。
 *
 *  仿真算法仍只使用仿真时间。本模块仅用于 SIL_REALTIME 的墙钟节拍和性能诊断，
 *  不得把墙钟值反馈到动力学、传感器或飞控算法。
 */
#ifndef COMMON_REALTIME_PACER_H
#define COMMON_REALTIME_PACER_H

#include "common/status.h"

#include <stdint.h>
#include <time.h>

/** @brief 一个实例私有的实时节拍状态。 */
typedef struct RealtimePacer {
    int enabled;
    double period_s;
    struct timespec next_deadline;
    uint64_t wait_count;
    uint64_t overrun_count;
    double total_sleep_s;
    double max_overrun_s;
} RealtimePacer;

/** @brief 读取 CLOCK_MONOTONIC 秒值；失败返回错误。 */
SimStatus monotonic_time_now(double *seconds_out);

/** @brief 初始化节拍器；period_s 必须为有限正数。 */
SimStatus realtime_pacer_init(RealtimePacer *pacer, double period_s, int enabled);

/** @brief 等待下一个绝对截止时间，并返回本次睡眠和超限秒数。 */
SimStatus realtime_pacer_wait_next(
    RealtimePacer *pacer,
    double *sleep_s_out,
    double *overrun_s_out);

#endif
