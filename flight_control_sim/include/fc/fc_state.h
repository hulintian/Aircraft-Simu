/** @file fc_state.h
 *  @brief 飞控运行状态快照。
 */
#ifndef FC_FC_STATE_H
#define FC_FC_STATE_H

#include "common/protocol.h"

/** @brief 飞控内部状态缓存。 */
typedef struct FcState {
    /** @brief 最近一次接收的传感器帧。 */
    SensorFrame last_sensor;
    /** @brief 最近一次生成的控制指令。 */
    ControlCommand last_command;
} FcState;

#endif
