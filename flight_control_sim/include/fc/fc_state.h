/** @file fc_state.h
 *  @brief 飞控运行状态快照和主控制器接口。
 *
 *  一个 @c FlightController 对象只属于一个飞行实例，内部保存该实例的
 *  模式、健康状态、估计器、制导管理器和命令管理器。对象不持有外部
 *  文件或网络资源，因此可被单元测试直接驱动。
 */
#ifndef FC_FC_STATE_H
#define FC_FC_STATE_H

#include "common/protocol.h"
#include "common/status.h"
#include "fc/autopilot.h"
#include "fc/command_manager.h"
#include "fc/estimator.h"
#include "fc/fc_health.h"
#include "fc/fc_modes.h"
#include "fc/fc_scheduler.h"
#include "fc/guidance_manager.h"
#include "fc/safety_monitor.h"

/** @brief 飞控内部状态缓存。 */
typedef struct FcState {
    /** @brief 最近一次接收的传感器帧。 */
    SensorFrame last_sensor;
    /** @brief 最近一次生成的控制指令。 */
    ControlCommand last_command;
} FcState;

/** @brief 飞控控制器运行配置。 */
typedef struct FlightControllerConfig {
    GuidancePngConfig guidance;
    FcSafetyConfig safety;
    double scheduler_base_rate_hz;
} FlightControllerConfig;

/** @brief 单实例飞控控制器对象。 */
typedef struct FlightController {
    FcState state;
    FcMode mode;
    FcHealth health;
    FcScheduler scheduler;
    Estimator estimator;
    GuidanceManager guidance;
    Autopilot autopilot;
    CommandManager command_manager;
    SafetyMonitor safety;
    uint32_t last_seq;
    int have_seq;
    double last_sensor_time;
    int have_last_sensor_time;
} FlightController;

/** @brief 初始化飞控控制器。 */
SimStatus flight_controller_init(
    FlightController *controller,
    const FlightControllerConfig *config);

/** @brief 使用一帧传感器数据推进飞控控制器并生成控制命令。 */
SimStatus flight_controller_step(
    FlightController *controller,
    const SensorFrame *sensor,
    ControlCommand *command);

#endif
