---
name: missile-sim-development
description: Maintain and extend this repository's C11 flight-control and environment closed-loop simulation. Use when auditing implementation progress, implementing items from docs/implementation_plan.md, modifying environment or flight-control models, adding sensor/guidance/multi-instance features, changing baseline JSON, updating project documentation, or running build and closed-loop verification.
---

# 飞控闭环仿真开发

## 读取项目事实

从仓库根目录工作。开始任务前按需读取：

1. `README.md`：当前进度、运行方式和已知限制。
2. `docs/design.md`：系统架构、数学模型、协议和工程约束。
3. `docs/implementation_plan.md`：P0-P8 顺序、模块和验收条件。
4. `configs/baseline/*.json`：当前可执行基线。
5. 目标模块的头文件、实现、CMake 和测试。
6. `git status --short`：识别用户已有修改，不覆盖无关改动。

不要只根据文件名或头文件存在判断功能完成。区分：

- **占位**：只有声明、空结构或未被构建。
- **独立实现**：模块有实现或单元测试，但未进入主链路。
- **主链路接入**：运行程序实际调用该模块。
- **阶段完成**：实现计划中的验收项全部通过。

报告进度时给出上述证据和未完成项。百分比只能作为粗略工程估算。

## 遵守架构约束

始终保持以下规则：

- 使用 C11 和现有 CMake 结构。
- 一个实例由一个 `environment_sim` 进程和一个 `flight_control_sim` 进程组成。
- 实例之间不通信，不共享可变运行时状态；仅共享只读程序、配置和地图资源。
- 每个实例使用独立 `instance_id`、UDP 端口和输出目录。
- 环境真值主坐标使用 ECEF；配置和地图输入使用 LLA/WGS-84。
- 配置使用完整合法 JSON，并校验 `schema_version` 和必填字段。
- 网络协议使用显式固定小端序列化、版本检查、实例检查和 CRC；不要直接发送 C 结构体内存。
- LOCKSTEP 下每个有效传感器帧对应一条控制指令。
- 主仿真循环避免不可控动态内存分配。
- 所有配置、网络和浮点输入均执行范围、有限值和状态校验。
- 固定种子运行必须可重复；随机状态必须属于实例。
- 公共接口使用 Doxygen 风格中文文档注释，写明单位、坐标系、所有权、错误条件和数值假设。

## 选择实现任务

用户要求“继续实现”时：

1. 核对 README 的进度结论是否与源码一致。
2. 找到最早一个未满足的阶段验收项。
3. 优先完成可进入主链路并可验证的纵向功能，不批量创建占位文件。
4. 保持修改集中在所属模块，复用 `common` 中已有数学、配置、协议和日志接口。
5. 在修改共享协议或配置结构前，检查环境端、飞控端、管理器、测试和基线配置的兼容影响。

默认阶段顺序：

```text
P0 工程骨架
P1 common
P2 协议与配置
P3 单实例闭环
P4 地球与地形
P5 环境与传感器
P6 飞控任务、制导与保护
P7 多实例管理器
P8 回放与批量验证
```

除非用户明确要求，不跨过前一阶段关键验收缺口去扩展后一阶段。

## 实现环境模型

在 `environment_sim` 中保持模型链路清晰：

```text
场景/目标
-> 大气、重力、质量、推进、气动
-> 执行机构
-> force_b / moment_b
-> 6DOF 积分
-> ECEF/LLA/AGL 真值
-> 传感器误差、延迟、丢包和故障
-> SensorFrame
```

不要将飞控指令直接写入位置。若暂时采用加速度级接口，必须明确标记为虚拟执行机构，并在 README 中说明尚未形成完整力/力矩链路。

动力学修改至少验证：

- 质量和惯量为正且有限。
- 四元数保持归一化。
- DCM 正交性和坐标方向正确。
- Euler/RK2/RK4 在固定输入下结果稳定且可重复。
- 地形缺失策略、AGL、碰撞和 LOS 遮挡符合配置。

## 实现传感器与飞控

传感器输出不得直接复制理想真值后宣称完成。按配置组合：

- 真值测量函数。
- 固定偏置、比例因子、白噪声和随机游走。
- 采样率、时间戳、延迟环形缓冲区、丢包和有效位。
- 按实例种子初始化的随机状态。
- 按仿真时间触发的故障。

飞控链路按以下结构推进：

```text
SensorFrame
-> 协议与时序校验
-> 导航/估计
-> 模式与健康状态机
-> 三维比例导引
-> 自动驾驶仪/指令管理
-> 幅值和变化率限制
-> 安全保护
-> ControlCommand
```

三维比例导引使用设计文档定义的形式，并验证 LOS 角速度叉乘方向。除幅值限幅外，还要实现变化率限制、旧帧拒绝、超时、无效测量和 NaN/Inf 降级处理。

## 实现多实例

`runtime.json` 中的 `instances[]` 是逐实例执行计划。管理器最终必须读取每项的：

- `instance_id`
- 场景、飞控和故障配置路径
- 随机种子
- 启用状态或其他计划字段

支持并验证串行与并行调度、并发上限、端口唯一性、输出隔离和
`CONTINUE_ON_FAILURE`。单实例失败不得终止其他实例。

开发基线保持小规模；大规模批次使用单独运行配置，不把 120 实例作为默认测试。

## 验证修改

每次实现后按风险选择测试，默认至少执行：

```sh
CCACHE_DISABLE=1 cmake --build build
CCACHE_DISABLE=1 ctest --test-dir build --output-on-failure
```

新增模块时增加聚焦单元测试；修改协议、配置、主循环、动力学、制导或进程管理时，同时运行闭环集成测试。

检查生成结果时至少确认：

- `run_manifest.json`
- `event_log.txt`
- `sensor_log.bin`
- `command_log.bin`
- `trajectory.csv`
- `summary.json`
- 多实例任务的 `campaign_summary.json`

不要把“编译通过”当作阶段完成。验收还应覆盖行为、错误路径、可重复性和跨进程闭环。

## 更新文档和进度

功能、配置、命令、输出格式或阶段状态变化后更新 `README.md`。进度表应明确：

- 已完成的验收项。
- 已独立实现但未接入主链路的模块。
- 当前仿真中的替代模型或限制。
- 尚缺的测试和数据资源，例如真实 DEM。
- 下一步最优先的实现项。

最终汇报保持简洁，包含修改文件、行为变化、测试结果、当前阶段和下一项工作。若测试因环境限制未运行，明确说明。
