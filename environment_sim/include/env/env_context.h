/** @file env_context.h
 *  @brief 环境进程启动上下文。
 */
#ifndef ENV_ENV_CONTEXT_H
#define ENV_ENV_CONTEXT_H

#include <stdint.h>

/** @brief 环境仿真进程的启动参数。 */
typedef struct EnvContext {
    /** @brief 当前实例编号。 */
    uint32_t instance_id;
    /** @brief 场景配置文件路径。 */
    const char *scenario_path;
    /** @brief 运行时配置文件路径。 */
    const char *runtime_path;
    /** @brief 故障脚本配置文件路径。 */
    const char *faults_path;
} EnvContext;

#endif
