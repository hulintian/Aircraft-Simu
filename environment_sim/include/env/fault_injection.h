/** @file fault_injection.h
 *  @brief 故障注入配置。
 */
#ifndef ENV_FAULT_INJECTION_H
#define ENV_FAULT_INJECTION_H

/** @brief 故障注入开关。 */
typedef struct FaultInjection {
    /** @brief 是否启用故障注入。 */
    int enabled;
} FaultInjection;

#endif
