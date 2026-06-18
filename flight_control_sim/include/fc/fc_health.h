/** @file fc_health.h
 *  @brief 飞控健康状态与告警标志。
 */
#ifndef FC_FC_HEALTH_H
#define FC_FC_HEALTH_H

#include <stdint.h>

/** @brief 飞控健康状态摘要。 */
typedef struct FcHealth {
    /** @brief 故障标志集合。 */
    uint32_t fault_flags;
    /** @brief 告警标志集合。 */
    uint32_t warning_flags;
} FcHealth;

#endif
