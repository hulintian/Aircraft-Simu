# 项目框架与当前实现

> 更新日期：2026-06-24
> 本文描述仓库中的实际代码结构。目标架构参见
> [design.md](design.md)，阶段任务参见
> [implementation_plan.md](implementation_plan.md)。

## 1. 项目定位

本项目是使用 C11 和 CMake 构建的软件在环闭环仿真系统。一个飞行实例由
两个独立进程组成：

```text
FlightInstance
├── environment_sim       环境真值、地形、动力学、传感器帧和运行记录
└── flight_control_sim    传感器接收、制导计算和控制指令发送
```

两个进程通过本机 UDP 交换固定小端二进制报文。环境进程拥有仿真时间，
当前采用一帧传感器对应一帧控制指令的锁步方式推进。

仓库还提供 `instance_manager`，用于启动多个相互隔离的进程对并汇总退出状态。

## 2. 总体分层

```text
                           tools/instance_manager
                           启动、回收、汇总实例
                                    │ exec
                 ┌──────────────────┴──────────────────┐
                 │                                     │
        environment_sim                        flight_control_sim
        环境主循环与记录                         飞控 UDP 主循环
                 │                                     │
        missile_environment                    missile_flight_control
        地球/地形/动力学模型                   状态机/估计/PNG/保护
                 └──────────────────┬──────────────────┘
                                    │
                             missile_common
                  数学、配置、协议、CRC、日志和通用状态码
```

CMake 当前生成以下主要目标：

| 目标 | 类型 | 说明 |
|---|---|---|
| `missile_common` | 静态库 | 全工程公共基础能力 |
| `missile_environment` | 静态库 | 环境和飞行器模型 |
| `missile_flight_control` | 静态库 | 飞控状态机、估计、制导、命令管理和保护 |
| `environment_sim` | 可执行程序 | 环境进程 |
| `flight_control_sim` | 可执行程序 | 飞控进程 |
| `instance_manager` | 可执行程序 | 多实例编排工具 |
| `common_tests` | 测试程序 | 公共库单元测试 |
| `environment_tests` | 测试程序 | 环境模型单元测试 |
| `flight_control_tests` | 测试程序 | 飞控 P6 单元测试 |
| `closed_loop_test` | 测试程序 | 双进程闭环集成测试 |

## 3. 目录职责

```text
common/
  include/common/        公共 API
  src/                   数学、配置、协议和日志实现
  tests/                 common 单元测试

environment_sim/
  include/env/           环境模型和环境进程 API
  src/                   环境主循环、地理/地形和动力学模型
  tests/                 环境模型单元测试

flight_control_sim/
  include/fc/            飞控状态、任务和算法接口
  src/                   飞控主循环、状态机、估计、三维 PNG、命令管理和保护
  tests/                 飞控单元测试

tools/instance_manager/  已实现的多实例进程管理器
tools/batch_runner/      预留
tools/log_convert/       预留
tools/map_preprocess/    预留
tools/replay/            预留

configs/baseline/        场景、飞控、运行时和故障基线配置
tests/                   跨进程闭环测试
scripts/                 Python 轨迹绘图脚本
docs/                    设计、计划和当前实现文档
runs/                    仿真产物，已被 Git 忽略
```

## 4. 单实例闭环

### 4.1 进程数据流

```text
environment_sim
  1. 读取 scenario.json 和 runtime.json
  2. 初始化 ECEF/LLA 状态、6DOF 状态、执行机构和地形
  3. 由当前真值构造 SensorFrame
  4. 编码、写日志并通过 UDP 发送
                     │
                     ▼
flight_control_sim
  5. 解码并校验协议、实例号和序列号
  6. 执行安全监视、导航估计、模式状态机、三维 PNG 和虚拟自动驾驶仪
  7. 执行命令保持、幅值/变化率限制，填充 ControlCommand 状态位并通过 UDP 返回
                     │
                     ▼
environment_sim
  8. 解码控制指令并写日志
  9. 执行机构一阶响应、限幅和速率限制
 10. 汇总虚拟控制力、气动、推进、重力和地球自转项
 11. 使用 Euler/RK2/RK4 推进 6DOF 状态并更新质量/惯量
 12. 更新目标匀速运动、LLA、AGL、碰撞和命中判断
 13. 写轨迹和摘要，然后进入下一仿真步
```

### 4.2 当前动力学边界

环境主循环已经使用 `PlantState6Dof`、统一环境力模型和 6DOF 积分器：

- 飞控输出的是 ECEF 加速度指令。
- 三个独立虚拟执行机构对三轴加速度执行一阶响应、位置限幅和速率限制。
- `environment_force_model` 把虚拟控制力、气动力和推进力汇总为 `force_b`，
  把气动力矩汇总为 `moment_b`，并提供 ECEF 重力和地球自转输入。
- 推进剂按质量流量消耗；推进剂不足时按本步可用质量缩放推力和流量。
- 惯量当前按总质量比例近似变化，尚未实现质心迁移和完整惯量张量演化。
- 基线启用重力、ISA 大气、低阻力气动和地球自转，推进模型默认关闭。

飞控指令仍是加速度级虚拟接口，气动控制面偏角当前为零。P6 已提供虚拟自动驾驶仪
和命令管理层，但真实姿态环与控制分配尚未完成，因此该链路用于验证环境物理项和
6DOF 软件结构，不代表真实舵面闭环。

### 4.3 当前传感器边界

`SensorFrame` 已由专用传感器模型生成：

- `sensor_imu`：机体系三轴角速度。
- `sensor_accel`：ECEF 三轴运动学加速度。
- `sensor_speed`：ECEF 三轴速度。
- `sensor_seeker`：距离、ECEF LOS 单位向量、LOS 角速度和闭合速度。
- 大地坐标、高度和 AGL 当前仍直接来自地理派生状态。

四类模型支持固定偏置、白噪声、随机游走、量化、限幅、采样保持、固定延迟和
整帧丢包。延迟线使用实例对象内的固定容量环形缓冲区，主循环不分配内存。
导引头 LOS 加噪后会重新归一化。基线导引头延迟为 20 ms。

环境程序可使用管理器传入的逐实例显式随机种子；没有传入时仍使用
`campaign.base_random_seed + instance_id` 派生种子，再为各传感器派生独立随机流。
飞控检查 `sensor_valid_flags`；导引头延迟预热、丢包或
地形遮挡时仍按 LOCKSTEP 返回受控零指令。基础 `faults.json` 已接入，
可按仿真时间触发传感器偏置、传感器强制无效/丢包、虚拟执行机构卡滞和
命令缩放，并把故障开始/恢复写入事件日志。单实例 `summary.json` 会记录
故障配置数量、触发/恢复次数、激活步数和传感器/执行机构影响步数。

### 4.4 地球与地形

已实现：

- WGS-84 椭球参数。
- LLA/ECEF 双向转换。
- ECEF 到 ENU/NED 局部坐标变换。
- 固定小端地形瓦片格式、CRC 和文件读写。
- DEM 双线性插值、AGL、地表碰撞和 LOS 遮挡采样。

当前 `env_app` 初始化地形时没有加载 `scenario.json` 中配置的瓦片路径，
而是以零个瓦片运行。基线配置使用 `FLAT_FILL`，因此当前地形等效为零椭球高
平面；真实 DEM 数据和地图预处理工具尚未提供。

## 5. 飞控框架

当前已进入执行链的飞控逻辑为：

```text
UDP SensorFrame
  -> 协议/实例号校验
  -> safety_monitor
  -> estimator/navigation
  -> fc_modes/fc_health
  -> guidance_manager/guidance_png_update
  -> autopilot
  -> command_manager
  -> 加速度幅值限幅和变化率限制
  -> UDP ControlCommand
```

已经实现：

- 三维比例导引。
- 输入有限值、距离、闭合速度和旧帧检查。
- `FC_POWER_ON` 到 `FC_SHUTDOWN` 的模式枚举和单帧状态机转换。
- 导航估计层，把速度、加速度、陀螺仪、大地坐标和导引头测量统一成 `NavState`。
- 制导管理器、虚拟自动驾驶仪和命令管理器。
- 加速度幅值限制和变化率限制。
- 无效导引头、超时、旧帧和 NaN/Inf 的保持/降级/故障状态位。
- `command_mode` 写入飞控模式，`command_status` 写入保护动作。
- `flight_control_tests` 独立覆盖 PNG 方向、异常输入、状态机、命令限制和保护动作。
- `closed_loop_test` 解码 `command_log.bin`，覆盖导引头延迟预热下的保持命令和有效测量后的变化率限制。

尚未接入或仅有接口定义：

- 严格的多速率调度执行；当前仍由 LOCKSTEP 传感器帧驱动。
- 真实姿态环、角速度环和舵面/执行机构控制分配。
- 更完整的飞控内部持久化日志和长时间故障恢复策略。

`flight_control.json` 中的 `scheduler.base_rate_hz`、`guidance.max_accel_rate_mps3`
和 `safety` 配置已经被飞控控制器读取和使用。`scheduler.tasks[]` 当前仍是任务表
设计预留，主循环没有按多速率任务拆分执行。

## 6. 公共基础库

`missile_common` 是两个进程和工具层的共同依赖：

| 模块 | 职责 |
|---|---|
| `status` | 统一 `SimStatus` 错误语义 |
| `vec3`、`matrix3`、`quaternion` | 三维数学和姿态运算 |
| `sim_time` | 仿真时钟基础类型 |
| `random` | 可重复伪随机数和正态分布采样 |
| `ring_buffer` | 固定容量环形缓冲区 |
| `crc32` | 报文和瓦片完整性校验 |
| `protocol` | `PacketHeader`、`SensorFrame`、`ControlCommand` |
| `packet` | 固定小端线格式编解码 |
| `config` | 轻量 JSON 语法、路径读取、版本和必填节校验 |
| `logger` | 当前最小标准输出日志封装 |

协议不直接发送 C 结构体内存，而是逐字段编码，以避免结构体填充和主机字节序
差异。接收端校验 magic、协议版本、消息类型、实例号、载荷长度和 CRC32。

## 7. 配置与运行状态

### 7.1 配置文件

| 文件 | 当前用途 |
|---|---|
| `scenario.json` | 步长、终止条件、初始状态、积分器、地形、重力、大气、气动、推进和执行机构参数 |
| `flight_control.json` | PNG 参数、调度基准频率、安全保护阈值和命令变化率限制 |
| `runtime.json` | 端口、输出目录、实例数、调度策略、并发上限和逐实例计划 |
| `faults.json` | 环境程序故障脚本；当前支持基础传感器和虚拟执行机构故障 |

当前配置模块不是完整 JSON Schema 引擎。它会检查 JSON 语法、
`schema_version`、必填对象和调用方读取的字段类型/范围。

### 7.2 实例隔离

实例号同时用于：

- 协议报文校验。
- UDP 端口偏移。
- 实例输出目录命名。
- 多实例管理器中的进程状态归属。

端口计算为：

```text
environment_port = environment_base_port + 2 * instance_id
flight_control_port = flight_control_base_port + 2 * instance_id
```

## 8. 多实例管理器

`instance_manager` 当前支持：

- 最多 128 个实例。
- 读取 `instances[]` 中启用的逐实例计划。
- 逐实例场景、飞控、故障配置路径和显式随机种子。
- 按 `max_parallel_instances` 限制并发实例对数量。
- `PARALLEL` 与 `SEQUENTIAL` 调度模式。
- `CONTINUE_ON_FAILURE` 与 `STOP_ON_FAILURE` 基础失败策略。
- 通过 `runtime.tools.environment_program` 和 `runtime.tools.flight_control_program`
  配置环境/飞控子程序路径。
- 启动前校验实例号、端口唯一性和 UDP 端口可绑定性。
- 启动和回收环境、飞控子进程。
- 子进程失败时返回非零退出码。
- 写出 `campaign_summary.json`，并聚合成功实例的命中、最近点、故障统计、
  配置路径、端口和随机种子。

当前限制：

- 子程序路径默认值仍为 `./build/...`，但可在 `runtime.tools` 中覆盖。
- 启动环境前会等待飞控 UDP 端口就绪，并识别飞控绑定前早退。
- 尚无应用层就绪/心跳握手。
- `failure_strategy` 的基础行为已实现，但端口占用和停止策略还需要更细测试。

管理器会把 `instances[]` 中的 `random_seed` 传给环境进程，因此同一批次可按计划
复现，不同实例不会共享传感器随机流。

## 9. 运行产物

环境进程负责写出单实例产物：

```text
runs/<campaign>/
  instance_0000/
    run_manifest.json
    event_log.txt
    sensor_log.bin
    command_log.bin
    trajectory.csv
    summary.json
```

多实例管理器额外写出：

```text
runs/<campaign>/campaign_summary.json
```

二进制日志保存完整协议线报文，可作为后续回放工具的输入基础，但当前没有
实现读取这些日志的回放程序。

## 10. 测试框架

当前 CTest 有五个测试：

| 测试 | 覆盖范围 |
|---|---|
| `common_tests` | 数学、四元数、随机数、环形缓冲区、配置和协议 |
| `environment_tests` | 坐标、地形瓦片、LOS、执行机构、环境模型、6DOF 和噪声 |
| `flight_control_tests` | PNG 方向、状态机、命令限幅/变化率限制和保护动作 |
| `closed_loop_test` | 双进程 UDP 锁步、命中结果和主要运行产物 |
| `instance_manager_test` | 两实例计划解析、子程序路径配置、端口隔离、端口占用预检、显式随机种子、飞控就绪等待、飞控早退失败路径、`STOP_ON_FAILURE` 跳过路径和 `campaign_summary.json` |

当前主要测试缺口：

- 没有飞控姿态环、控制分配和多速率任务调度测试。
- 没有实例管理器应用层就绪/心跳握手集成测试。
- 没有真实 DEM、回放一致性和 Monte Carlo 统计测试。
- 没有性能、实时性、覆盖率或 sanitizer 测试配置。

## 11. 当前完成度

以下进度按 `implementation_plan.md` 的交付项估算，重点区分“代码存在”和
“已接入闭环”：

| 阶段 | 进度 | 判断 |
|---|---:|---|
| P0 工程骨架 | 100% | 构建、严格警告、CTest 和三个主程序已具备 |
| P1 common | 100% | 计划内公共基础模块已实现并测试 |
| P2 协议与配置 | 100% | 固定线协议和轻量配置校验已进入主链路 |
| P3 单实例闭环 | 100% | 双进程锁步、日志和集成测试通过 |
| P4 地球与地形 | 90% | 算法和单测完整；真实瓦片加载链和预处理工具未接入 |
| P5 环境与传感器 | 98% | 环境力链、四类传感器误差/延迟/丢包、基础故障脚本、故障统计、错误路径和固定种子回归已集成；更多故障类型仍需扩展 |
| P6 飞控系统 | 75% | 静态库、状态机、估计、PNG、虚拟自动驾驶仪、命令管理、幅值/变化率限制、保护状态和测试已进入主链路；真实姿态环与控制分配未完成 |
| P7 多实例 | 92% | 逐实例配置/种子、并发/串行调度、子程序路径配置、端口预检、端口占用测试、飞控端口就绪等待、失败返回码、`STOP_ON_FAILURE` 跳过路径、任务摘要和集成测试已进入主链路；应用层就绪/心跳握手未完成 |
| P8 回放与验证 | 15% | 日志和绘图可用，回放和批量验证工具为空 |

以完整规划为目标，当前工程功能完成度约为 **89%**；以“可构建、可运行、
可命中的最小双进程闭环”为目标，核心 MVP 已完成。

## 12. 建议的后续顺序

1. 完成 P6 剩余的真实姿态环、控制分配、多速率任务执行和飞控内部持久化日志。
2. 为 `instance_manager` 增加应用层就绪/心跳握手。
3. 扩展更完整的故障类型库和真实 DEM 接入。
4. 实现二进制日志回放、日志转换、地图预处理和批量统计工具。

## 13. 需求描述覆盖矩阵

本节用于回答“需求中的飞行仿真环境、飞控程序和闭环协同机制是否已经在项目中使用”。
判断分为三类：

- 已进入主链路：运行程序实际调用，闭环测试会覆盖。
- 部分进入主链路：接口或简化模型已运行，但还不是目标高保真设计。
- 仅设计预留：文档或头文件存在，运行程序尚未使用。

| 需求项 | 当前状态 | 证据与限制 |
|---|---|---|
| C 语言双程序架构 | 已进入主链路 | `environment_sim` 和 `flight_control_sim` 是独立 C 可执行程序 |
| 环境程序作为被控对象 | 已进入主链路 | 环境进程拥有仿真时间、真值状态、传感器生成和轨迹记录 |
| 六自由度状态 | 已进入主链路 | `PlantState6Dof` 包含 ECEF 位置/速度、姿态四元数、角速度、质量、惯量、力和力矩 |
| 六自由度积分 | 已进入主链路 | `missile_plant_step` 支持 Euler、RK2 和 RK4，baseline 使用 RK4 |
| 重力模型 | 已进入主链路 | `environment_force_model` 计算并传入 ECEF 重力 |
| 大气模型 | 已进入主链路 | 当前为 ISA 对流层模型，含温度、压力、密度和声速 |
| 风速 | 已进入主链路 | 从 `scenario.json` 读取 ECEF 风速并用于相对气流 |
| 气动力/力矩 | 部分进入主链路 | 当前为二次阻力和线性舵效模型，还不是气动表插值 |
| 发动机推力 | 部分进入主链路 | 推进模型已接入力链，baseline 默认关闭 |
| 质量变化 | 已进入主链路 | 推进剂流量驱动质量变化，惯量按质量比例近似缩放 |
| 真实地图球形地球 | 部分进入主链路 | WGS-84、LLA/ECEF、AGL、LOS 已有；真实 DEM 瓦片加载链未接入 |
| IMU/陀螺仪 | 已进入主链路 | `sensor_imu` 输出机体系角速度测量 |
| 加速度计 | 已进入主链路 | `sensor_accel` 输出 ECEF 加速度测量 |
| 速度计 | 已进入主链路 | `sensor_speed` 输出 ECEF 速度测量 |
| 导引头测量 | 已进入主链路 | `sensor_seeker` 输出距离、LOS 单位向量、LOS 角速度和闭合速度 |
| 传感器噪声/延迟/丢包 | 已进入主链路 | 支持偏置、白噪声、随机游走、量化、采样保持、延迟和丢包 |
| 故障脚本 | 部分进入主链路 | 环境程序已读取 `faults.json` 并触发基础传感器/虚拟执行机构故障；单实例和成功批次统计已接入，更多故障类型仍未完成 |
| 比例导引法 | 已进入主链路 | `guidance_png_update` 使用三维 PNG 输出 ECEF 加速度指令 |
| 飞控状态机/调度器 | 部分进入主链路 | 模式状态机、调度器配置校验和 tick 视图已进入飞控静态库；主循环尚未按多速率任务表执行 |
| 自动驾驶仪/控制分配 | 部分进入主链路 | 虚拟自动驾驶仪已把 PNG 加速度转成标准命令；真实姿态环和舵面控制分配未完成 |
| UDP 闭环通信 | 已进入主链路 | 固定小端线协议、CRC、版本和 instance_id 校验已接入 |
| 共享内存通信 | 未使用 | 当前设计选择 UDP；共享内存可作为后续接口扩展 |
| 多实例独立运行 | 已进入主链路 | 管理器读取逐实例配置和显式种子，执行端口预检、飞控端口就绪等待并输出批次摘要；应用层心跳仍未完成 |
| GPU 并行化 | 未使用 | 当前为 CPU 多进程并行；GPU 属于 P8 之后性能扩展 |

对应的设计收敛基线见 [design.md 第 21 节](design.md#21-设计完成基线与落地边界)。
