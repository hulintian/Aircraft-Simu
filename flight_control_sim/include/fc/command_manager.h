/** @file command_manager.h
 *  @brief 控制指令管理器。
 */
#ifndef FC_COMMAND_MANAGER_H
#define FC_COMMAND_MANAGER_H

#include "common/protocol.h"

/** @brief 命令管理器缓存。 */
typedef struct CommandManager {
    /** @brief 最近一次输出的控制命令。 */
    ControlCommand last_command;
} CommandManager;

#endif
