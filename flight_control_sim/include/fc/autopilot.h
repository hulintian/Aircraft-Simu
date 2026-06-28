/** @file autopilot.h
 *  @brief 自动驾驶仪和指令层转换接口。
 *
 *  环境程序当前接收加速度级虚拟执行机构指令。自动驾驶仪模块把制导
 *  期望加速度转换为标准控制命令的加速度、姿态、角速度和执行机构通道。
 *  后续接入真实舵面控制时，可以在该层增加姿态环和角速度环。
 */
#ifndef FC_AUTOPILOT_H
#define FC_AUTOPILOT_H

#include "common/protocol.h"
#include "common/status.h"
#include "fc/guidance_png.h"

#include <stdint.h>

/** @brief 自动驾驶仪输出的标准化命令视图。 */
typedef struct AutopilotCommand {
    /** @brief ECEF 坐标系加速度期望，单位 m/s^2。 */
    Vec3 accel_cmd_ecef;
    /** @brief 预留姿态期望，当前为零向量。 */
    Vec3 attitude_cmd;
    /** @brief 预留机体系角速度期望，当前为零向量。 */
    Vec3 body_rate_cmd;
    /** @brief 预留执行机构通道，当前全部置零。 */
    double actuator_cmd[SIM_MAX_ACTUATORS];
} AutopilotCommand;

/** @brief 自动驾驶仪状态。 */
typedef struct Autopilot {
    uint32_t accepted_count;
    uint32_t rejected_count;
} Autopilot;

/** @brief 初始化自动驾驶仪状态。 */
SimStatus autopilot_init(Autopilot *autopilot);

/** @brief 将制导输出转换为自动驾驶仪命令。 */
SimStatus autopilot_update(
    Autopilot *autopilot,
    const GuidancePngOutput *guidance,
    AutopilotCommand *out);

/** @brief 生成受控零自动驾驶仪命令。 */
void autopilot_zero_command(AutopilotCommand *out);

#endif
