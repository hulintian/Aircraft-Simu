/** @file realtime_pacer.c
 *  @brief 单调墙钟实时节拍器实现。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/realtime_pacer.h"

#include <errno.h>
#include <math.h>
#include <string.h>

/** @brief 将 timespec 转换为秒。 */
static double timespec_seconds(struct timespec value)
{
    return (double)value.tv_sec + (double)value.tv_nsec * 1.0e-9;
}

/** @brief 给绝对时间增加有限正秒数。 */
static void timespec_add_seconds(struct timespec *value, double seconds)
{
    const time_t whole_seconds = (time_t)seconds;
    const long nanoseconds = (long)((seconds - (double)whole_seconds) * 1.0e9 + 0.5);

    value->tv_sec += whole_seconds;
    value->tv_nsec += nanoseconds;
    if (value->tv_nsec >= 1000000000L) {
        value->tv_sec += 1;
        value->tv_nsec -= 1000000000L;
    }
}

SimStatus monotonic_time_now(double *seconds_out)
{
    struct timespec now;

    if (seconds_out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return SIM_ERR_IO;
    }
    *seconds_out = timespec_seconds(now);
    return SIM_OK;
}

SimStatus realtime_pacer_init(RealtimePacer *pacer, double period_s, int enabled)
{
    if (pacer == 0 || !isfinite(period_s) || period_s <= 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(pacer, 0, sizeof(*pacer));
    pacer->enabled = enabled != 0;
    pacer->period_s = period_s;
    if (clock_gettime(CLOCK_MONOTONIC, &pacer->next_deadline) != 0) {
        return SIM_ERR_IO;
    }
    timespec_add_seconds(&pacer->next_deadline, period_s);
    return SIM_OK;
}

SimStatus realtime_pacer_wait_next(
    RealtimePacer *pacer,
    double *sleep_s_out,
    double *overrun_s_out)
{
    struct timespec now;
    double now_s;
    double deadline_s;
    double sleep_s = 0.0;
    double overrun_s = 0.0;

    if (pacer == 0 || sleep_s_out == 0 || overrun_s_out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return SIM_ERR_IO;
    }
    now_s = timespec_seconds(now);
    deadline_s = timespec_seconds(pacer->next_deadline);
    if (pacer->enabled != 0 && now_s < deadline_s) {
        int wait_status;

        do {
            wait_status = clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &pacer->next_deadline,
                0);
        } while (wait_status == EINTR);
        if (wait_status != 0) {
            return SIM_ERR_IO;
        }
        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
            return SIM_ERR_IO;
        }
        sleep_s = timespec_seconds(now) - now_s;
        if (sleep_s < 0.0) {
            sleep_s = 0.0;
        }
    } else if (pacer->enabled != 0) {
        overrun_s = now_s - deadline_s;
        ++pacer->overrun_count;
        if (overrun_s > pacer->max_overrun_s) {
            pacer->max_overrun_s = overrun_s;
        }
    }
    ++pacer->wait_count;
    pacer->total_sleep_s += sleep_s;
    timespec_add_seconds(&pacer->next_deadline, pacer->period_s);
    *sleep_s_out = sleep_s;
    *overrun_s_out = overrun_s;
    return SIM_OK;
}
