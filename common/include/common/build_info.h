/** @file build_info.h
 *  @brief 构建版本与协议版本常量。
 *
 *  该文件集中定义软件版本和通信协议版本，便于配置文件、日志和报文头
 *  使用同一组常量进行版本比对。
 */
#ifndef COMMON_BUILD_INFO_H
#define COMMON_BUILD_INFO_H

/** @brief 仿真软件主版本号。 */
#define MISSILE_SIM_VERSION_MAJOR 0
/** @brief 仿真软件次版本号。 */
#define MISSILE_SIM_VERSION_MINOR 1
/** @brief 仿真软件补丁版本号。 */
#define MISSILE_SIM_VERSION_PATCH 0

#ifndef MISSILE_SIM_GIT_COMMIT
#define MISSILE_SIM_GIT_COMMIT "unknown"
#endif
#ifndef MISSILE_SIM_GIT_DIRTY
#define MISSILE_SIM_GIT_DIRTY 0
#endif
#ifndef MISSILE_SIM_BUILD_TIME
#define MISSILE_SIM_BUILD_TIME "unknown"
#endif
#ifndef MISSILE_SIM_COMPILER
#define MISSILE_SIM_COMPILER "unknown"
#endif

/** @brief 通信协议主版本号。 */
#define MISSILE_SIM_PROTOCOL_VERSION_MAJOR 1
/** @brief 通信协议次版本号。 */
#define MISSILE_SIM_PROTOCOL_VERSION_MINOR 0

#endif
