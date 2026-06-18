/** @file fc_context.h
 *  @brief 飞控进程启动上下文。
 */
#ifndef FC_FC_CONTEXT_H
#define FC_FC_CONTEXT_H

#include <stdint.h>

/** @brief 飞控仿真进程的启动参数。 */
typedef struct FcContext {
    /** @brief 当前实例编号。 */
    uint32_t instance_id;
    /** @brief 飞控配置文件路径。 */
    const char *flight_control_path;
    /** @brief 运行时配置文件路径。 */
    const char *runtime_path;
} FcContext;

#endif
