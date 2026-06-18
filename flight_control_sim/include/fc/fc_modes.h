/** @file fc_modes.h
 *  @brief 飞控工作模式枚举。
 */
#ifndef FC_FC_MODES_H
#define FC_FC_MODES_H

/** @brief 飞控状态机模式。 */
typedef enum FcMode {
    FC_POWER_ON = 0,
    FC_SELF_TEST,
    FC_WAIT_SENSOR,
    FC_NAV_READY,
    FC_GUIDANCE_STANDBY,
    FC_GUIDANCE_ACTIVE,
    FC_COMMAND_HOLD,
    FC_DEGRADED,
    FC_FAULT,
    FC_SHUTDOWN
} FcMode;

#endif
