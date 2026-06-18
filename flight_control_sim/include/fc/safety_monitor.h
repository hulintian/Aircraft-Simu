/** @file safety_monitor.h
 *  @brief 飞控安全监视状态。
 */
#ifndef FC_SAFETY_MONITOR_H
#define FC_SAFETY_MONITOR_H

#include <stdint.h>

/** @brief 安全监视器状态。 */
typedef struct SafetyMonitor {
    /** @brief 当前健康/故障标志。 */
    uint32_t status_flags;
} SafetyMonitor;

#endif
