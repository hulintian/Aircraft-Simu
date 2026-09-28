# 项目框架与当前实现

> 更新日期：2026-08-20
> 本文描述仓库中的实际代码结构。目标架构参见
> [design.md](design.md)，阶段任务参见
> [implementation_plan.md](implementation_plan.md)。

## 0. 阅读路径

新成员建议按以下顺序阅读和操作：

1. [onboarding.md](onboarding.md)：先完成构建、CTest、单实例和多实例运行。
2. 本文：理解当前源码实际分层、数据流、配置、日志和验收测试。
3. [tools.md](tools.md)：需要处理日志、回放、比较、批量统计、地图预处理或批跑时阅读。
4. [design.md](design.md)：理解目标架构和长期边界。
5. [implementation_plan.md](implementation_plan.md)：按 P0-P8 阶段定位历史任务与剩余增强项。
6. [flight_sim_software_comparison.md](flight_sim_software_comparison.md)：理解本项目与成熟飞行仿真/任务工程平台的能力差距。

如果只想快速改一个功能，先读 `onboarding.md` 的“常见修改路径”，再回到本文对应章节。

## 1. 项目定位

本项目是使用 C11 和 CMake 构建的软件在环闭环仿真系统。一个飞行实例由
两个独立进程组成：

```text
FlightInstance
├── environment_sim       环境真值、地形、动力学、传感器帧和运行记录
└── flight_control_sim    传感器接收、制导计算和控制指令发送
```

两个进程通过本机 UDP 交换固定小端二进制报文。环境进程拥有仿真时间，
回归默认采用一帧传感器对应一帧控制指令的 `LOCKSTEP`；实时演示和网络故障恢复可使用
`FREE_RUNNING`。墙钟执行可选 `SIL_FAST` 或带单调绝对时间节拍的 `SIL_REALTIME`。

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
| `log_convert` | 可执行程序 | 二进制协议日志到 CSV 转换工具 |
| `replay` | 可执行程序 | 使用传感器日志离线驱动飞控 |
| `compare_logs` | 可执行程序 | 对比传感器/控制协议日志或真值轨迹 CSV |
| `batch_stats` | 可执行程序 | 汇总单实例或批次摘要 |
| `map_preprocess` | 可执行程序 | 裸 ASCII/ESRI ASCII Grid 高程网格到内部地形瓦片预处理工具 |
| `batch_runner` | 可执行程序 | 批次清单入口，顺序调用实例管理器并可选聚合统计 |
| `control_quality_report` | 可执行程序 | 标准机动控制品质验收与 JSON 报告 |
| `tools/plot/*.py` | Python 工具 | 轨迹、状态时序、数值诊断和批次统计 PNG 绘图 |
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
tools/log_convert/       已实现的二进制协议日志到 CSV 转换工具
tools/replay/            已实现的传感器日志回放工具
tools/compare_logs/      已实现的协议日志/真值轨迹比较工具
tools/batch_stats/       已实现的摘要聚合工具
tools/map_preprocess/    已实现的裸 ASCII/ESRI ASCII Grid 到内部地形瓦片预处理工具
tools/batch_runner/      已实现的批次清单运行入口
tools/control_quality/   已实现的标准机动控制品质报告工具
tools/plot/              已实现的 Python 绘图工具

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
 12. 更新目标匀速/脚本加速度机动、LLA、AGL、碰撞和步间连续命中判断
 13. 写轨迹和摘要，然后进入下一仿真步
```

### 4.2 当前动力学边界

环境主循环已经使用 `PlantState6Dof`、统一环境力模型和 6DOF 积分器：

- 飞控输出的是 ECEF 加速度指令。
- 三个独立虚拟执行机构对三轴加速度执行一阶响应、位置限幅和速率限制。
- `environment_force_model` 把虚拟控制力、气动力和推进力汇总为 `force_b`，
  把气动力矩汇总为 `moment_b`，并提供 ECEF 重力和地球自转输入。
- 推进剂按质量流量消耗；推进剂不足时按本步可用质量缩放推力和流量。
- 可选质量属性模型按推进剂比例更新质心和完整 3x3 惯量张量，并使用平行轴定理合成总惯量；旧配置保留质量比例兼容路径。
- 基线启用重力、ISA 大气、低阻力气动和地球自转，推进模型默认关闭。
- 风模型支持基础 ECEF 风、高度切变、正弦阵风和固定种子的一阶高斯-马尔可夫湍流。
- `target_model` 支持匀速和脚本 ECEF 常加速度段，步内开始/结束边界分段积分；
  `hit_detect` 对相对线段求连续最近点，避免高速穿越漏判。
- `aero_database` 模块已支持版本化文件、CRC、气动表样本插值和包络外策略测试；
  环境程序可通过 `aerodynamics.table_path` 进入气动主链路。baseline 未配置该路径时
  仍使用简化气动模型。
- `aero_database_v2` 支持 Mach、迎角、侧滑角、高度、俯仰舵偏和偏航舵偏六维规则网格、
  64 角多线性插值、CRC/单位校验和三种包络外策略；`table_v2_path` 已由闭环测试覆盖。

飞控指令仍保留加速度级虚拟接口，作为当前闭环的主等效控制力。P6 已提供
姿态/角速度自动驾驶仪和控制分配层，飞控会填充 `attitude_cmd`、`body_rate_cmd`
以及俯仰/偏航舵偏，环境气动模型会消费这些舵偏并形成附加气动力/力矩。该链路
用于验证环境物理项和 6DOF 软件结构，仍不代表真实弹载飞控。

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
可按仿真时间触发传感器偏置/漂移/噪声增大/卡滞/延迟/饱和，执行机构卡滞/偏置/
速率或行程退化/延迟/失能，以及报文丢弃/延迟/重复/损坏/乱序，并把故障开始/恢复
写入事件日志。单实例 `summary.json` 会记录
故障配置数量、触发/恢复次数、激活步数和传感器/执行机构影响步数。

### 4.4 地球与地形

已实现：

- WGS-84 椭球参数。
- LLA/ECEF 双向转换。
- ECEF 到 ENU/NED 局部坐标变换。
- 固定小端地形瓦片格式、CRC 和文件读写。
- DEM 双线性插值、AGL、地表碰撞和 LOS 遮挡采样。
- `map.tile_path`、`map.tile_paths[]` 和 `map.tile_index_path` 文本/二进制空间索引瓦片加载链，索引内相对瓦片路径按索引文件所在目录解析，闭环测试会生成临时瓦片并通过二进制索引实际加载。
- `map.terrain.cache_tile_count` 固定槽位 LRU 懒加载/淘汰缓存、manifest 追踪和 summary 缓存统计。

当前 baseline 未配置 `map.tile_path`，并仍使用 `FLAT_FILL`，因此默认地形等效为零椭球高平面。
`map_preprocess` 已能把裸 ASCII 和 ESRI ASCII Grid 高程网格写成内部瓦片格式，并可用
`--index-output` 写出二进制空间索引；`map.tile_path`、`map.tile_paths[]` 和文本/二进制
`map.tile_index_path` 可加载内部瓦片；`--nodata-fill` 显式缺测填补已接入，运行时固定槽位
LRU 懒加载/淘汰缓存已接入。仓库中的珠峰附近 9x9 真实来源 fixture 带来源和 SHA-256 清单，
预处理测试覆盖峰值与 LOS，双进程闭环直接加载其派生瓦片并验证遮挡保护。

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
- `flight_control_tests` 独立覆盖 PNG 方向、异常输入、状态机、命令限制、连续帧控制品质边界和保护动作。
- `closed_loop_test` 解码 `command_log.bin`，覆盖导引头延迟预热下的保持命令和有效测量后的变化率限制。
- 飞控进程写出 `fc_internal_log.bin`，记录每帧模式、状态和主要控制中间量。

工程基线还包括 `control_quality_report` 的阶跃、指令反向和丢包恢复标准机动，以及
`log_convert --type fc-internal` 的正式内部日志解码。真实型号稳定裕度、带宽和飞行品质
仍需 V6/V7 外部模型与试验数据，不能由该工程报告替代。

`flight_control.json` 中的 `scheduler.base_rate_hz`、`guidance.max_accel_rate_mps3`
和 `safety` 配置已经被飞控控制器读取和使用。飞控主循环由收到的传感器帧驱动，环境可
使用 `LOCKSTEP` 或 `FREE_RUNNING`。导航、制导和控制任务按 `scheduler.tasks[]` 的整数
tick 周期执行；非到期 tick 会复用最近一次有效结果。

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
| `provenance` | 文件 CRC/大小、UTC 时间和安全 JSON 字符串写入 |
| `realtime_pacer` | 单调时钟节拍、计算耗时、超限和软实时裕度统计 |
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

当前配置模块不是完整 JSON Schema 引擎。它会检查 JSON 语法、`schema_version`、必填对象、
调用方读取的字段类型/范围，并遍历主要配置对象记录未识别字段 warning；正式生产仍应把
当前白名单迁移为版本化机器可读 schema 和独立 schema 回归。

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
- 写出 `campaign_summary.json`，并聚合批次命中/未命中/超时、逐实例脱靶量
  min/max/mean/std、失败实例，以及成功实例的故障/数值诊断、配置路径、端口和随机种子。

当前限制：

- 子程序路径默认值仍为 `./build/...`，但可在 `runtime.tools` 中覆盖。
- 当前仅支持本机进程启动，尚无远程节点启动和资源配额调度。

管理器会把 `instances[]` 中的 `random_seed` 传给环境进程，因此同一批次可按计划
复现，不同实例不会共享传感器随机流。

## 9. 运行产物

环境进程负责写出单实例产物：

```text
runs/<campaign>/
  instance_0000/
    run_manifest.json
    config_snapshot_scenario.json
    config_snapshot_runtime.json
    config_snapshot_faults.json
    event_log.txt
    fc_run_manifest.json
    fc_config_snapshot_runtime.json
    config_snapshot_flight_control.json
    performance.json
    fc_performance.json
    fc_internal_log.bin
    sensor_log.bin
    command_log.bin
    trajectory.csv
    trajectory_diagnostics.csv
    summary.json
```

多实例管理器额外写出：

```text
runs/<campaign>/campaign_summary.json
```

二进制日志保存完整协议线报文。`tools/log_convert` 可把 `sensor_log.bin`、
`command_log.bin` 或固定 92 字节记录的 `fc_internal_log.bin` 转换为 CSV；`tools/replay` 可用 `sensor_log.bin`
离线驱动飞控并生成新的 `command_log.bin`；`tools/compare_logs` 可精确或按容差比较
传感器/控制日志及 `trajectory.csv`；`tools/batch_stats` 可聚合单实例和批次摘要，
输出失败实例和失败原因分布。
`log_convert`、`replay` 和 `batch_runner` 分别以 `REPLAY_PASSIVE`、`REPLAY_WITH_FC` 和
`MONTE_CARLO` 写出 sidecar `*.run_manifest.json`，记录输入/输出 CRC、软件身份、帧或样本数、
种子来源和工作流状态。

## 10. 测试框架

当前默认 CTest 有 11 个测试；medium 和 long 压力层显式启用：

| 测试 | 覆盖范围 |
|---|---|
| `common_tests` | 数学、四元数、随机数、环形缓冲区、配置和协议 |
| `environment_tests` | 坐标、地形、质量惯量、风、v1/v2 气动表、故障、环境力、6DOF 和噪声 |
| `flight_control_tests` | PNG 方向/限幅/NaN 拒绝、姿态/角速度自动驾驶仪、内环开关、控制分配、多速率调度、状态机、命令限幅/变化率限制、连续帧控制品质边界和保护动作 |
| `closed_loop_test` | 双进程 UDP 锁步、真实 DEM LOS、v1/v2 气动表、内部日志、诊断和真实日志回放比较 |
| `instance_manager_test` | 并发/串行两实例计划、子程序路径配置、端口隔离/占用预检、显式随机种子、应用层 ready 心跳、飞控早退、`CONTINUE_ON_FAILURE` 继续、`STOP_ON_FAILURE` 跳过路径和 `campaign_summary.json` |
| `log_convert_test` | sensor、command 和飞控内部日志到 CSV 的解码转换 |
| `p8_tools_test` | `replay`、二进制/轨迹精确与容差比较、`batch_stats` 脱靶/失败原因/诊断/气动 flags 聚合和截断日志失败路径 |
| `map_preprocess_test` | 裸 ASCII/ESRI/HGT、真实来源 DEM、CRC、插值和 LOS |
| `batch_runner_test` | 批次清单解析、`instance_manager --runtime` 顺序调用和 Monte Carlo runtime 模板展开 |
| `control_quality_test` | 阶跃、指令反向和丢包恢复的控制品质阈值 |
| `campaign_pressure_short_test` | 默认 4 实例、并发 2 的短压力运行 |
| `campaign_pressure_medium_test` | 可选 16 实例、并发 4 的中压力运行 |
| `campaign_pressure_long_test` | 可选 128 实例、并发 8 的长压力运行 |

当前仓库内测试边界：

- 已有真实来源最小 DEM fixture 和 4/16/128 压力分档；目标机器的真实大规模长时资源结论仍需独立 V6 基准。
- Git/构建身份、逐配置 CRC32/输入快照、运行模式、标准日志路径，以及气动表、地形资源和
  surrogate 模型路径/模型版本/训练数据版本/适用包线已进入 `run_manifest.json`；
  固定格式线性 surrogate 可只读推理并拒绝包线外输入，地形缓存统计已进入 `summary.json`。
- 已有覆盖率和 sanitizer CMake 配置；当前环境缺 ASan 运行库时 sanitizer 会在配置阶段失败。

## 11. 当前完成度

P0-P8 是 V0-V5 仓库内软件工程计划，当前验收入口均已实现并进入自动测试。它不是产品
完成百分比，不包含 V6/V7 真实数据标定、外部交叉验证和硬件试验。多轴成熟度和阻塞项见
[current_progress_and_gaps.md](current_progress_and_gaps.md)。

| 阶段 | 工程状态 | 判断 |
|---|---|---|
| P0-P3 | 完成 | 构建、公共库、协议配置和单实例双进程闭环均有自动验收 |
| P4 | 完成 | 地球/地形、磁盘瓦片 LRU、三类预处理、真实来源 DEM 和 LOS 闭环已覆盖 |
| P5 | 完成 | 6DOF、质量惯量、风、传感器和连续/周期/变延迟故障已进入主链路 |
| P6 | 完成 | 飞控链、正式内部日志解码和标准机动控制品质报告已覆盖 |
| P7 | 完成 | 逐实例编排、隔离、心跳、失败策略和摘要已覆盖 |
| P8 | 完成 | 回放、版本化容差、统计、绘图、批跑、v1/v2 气动和 4/16/128 压力层已覆盖 |

当前最高可声明为“V0-V5 软件在环工程基线完成”。真实型号性能、合格控制律和硬件实时性
仍未完成 V6/V7。

当前 `env_app.c` 和 `fc_app.c` 仍集中承担配置、网络、调度、运行记录和应用编排。其行为已有
测试，但在多人并行开发、替换真实接口或进入资格化流程前，应按既有模型 API 拆分应用服务，
并为 I/O、场景装载和记录器建立可替换边界；仓库不再保留没有实现和调用方的占位头文件。

## 12. 建议的后续顺序

1. 导入目标外形 CFD/风洞气动数据并完成 V6 外部交叉验证。
2. 用真实传感器、执行机构、推进和质量属性数据标定参数与包线。
3. 在目标机器执行长时资源基准，并接入真实总线/HIL 或台架完成 V7。
4. 非锁步通信若成为需求，先设计协议 v2 的独立时间戳和异步队列语义。

## 13. 需求描述覆盖矩阵

本节用于回答“需求中的飞行仿真环境、飞控程序和闭环协同机制是否已经在项目中使用”。
判断分为三类：

- 已进入主链路：运行程序实际调用，闭环测试会覆盖。
- 部分进入主链路：接口或简化模型已运行，但还不是目标高保真设计。
- 仅设计预留：设计文档定义了扩展方向，但运行程序尚未使用。

| 需求项 | 当前状态 | 证据与限制 |
|---|---|---|
| C 语言双程序架构 | 已进入主链路 | `environment_sim` 和 `flight_control_sim` 是独立 C 可执行程序 |
| 环境程序作为被控对象 | 已进入主链路 | 环境进程拥有仿真时间、真值状态、传感器生成和轨迹记录 |
| 六自由度状态 | 已进入主链路 | `PlantState6Dof` 包含 ECEF 位置/速度、姿态四元数、角速度、质量、惯量、力和力矩 |
| 六自由度积分 | 已进入主链路 | `missile_plant_step` 支持 Euler、RK2 和 RK4，baseline 使用 RK4 |
| 重力模型 | 已进入主链路 | `environment_force_model` 计算并传入 ECEF 重力 |
| 大气模型 | 已进入主链路 | 当前为 ISA 对流层模型，含温度、压力、密度和声速 |
| 风速 | 已进入主链路 | 基础 ECEF 风、高度切变、正弦阵风和固定种子一阶高斯-马尔可夫湍流用于相对气流 |
| 气动力/力矩 | 已进入工程主链路 | baseline 为低阶模型；v1 表、六维 v2 表和线性 surrogate 均可配置接入并写 manifest；真实数据可信度仍需 V6 |
| 发动机推力 | 部分进入主链路 | 推进模型已接入力链，baseline 默认关闭 |
| 目标机动 | 已进入工程主链路 | 匀速和脚本 ECEF 加速度段已实现并单测；尚无完整目标飞行动力学和行为库 |
| 连续命中判定 | 已进入主链路 | 相对步间线段求最近点，覆盖高速穿越；真实引信/毁伤模型未建立 |
| 质量变化 | 已进入主链路 | 推进剂流量驱动质量变化；可选模型更新质心和完整惯量张量，旧配置保留比例近似 |
| 真实地图球形地球 | 已进入工程主链路 | WGS-84、坐标、AGL、LOS、三类预处理、磁盘瓦片 LRU、真实来源 DEM fixture 和真实 DEM LOS 闭环已有；生产地图覆盖仍需外部资源 |
| IMU/陀螺仪 | 已进入主链路 | `sensor_imu` 输出机体系角速度测量 |
| 加速度计 | 已进入主链路 | `sensor_accel` 输出 ECEF 加速度测量 |
| 速度计 | 已进入主链路 | `sensor_speed` 输出 ECEF 速度测量 |
| 导引头测量 | 已进入主链路 | `sensor_seeker` 输出距离、LOS 单位向量、LOS 角速度和闭合速度 |
| 传感器噪声/延迟/丢包 | 已进入主链路 | 支持偏置、白噪声、随机游走、量化、采样保持、延迟和丢包 |
| 故障脚本 | 已进入工程主链路 | 覆盖设计中的传感器、执行机构和通信故障矩阵；器件参数与真实网络时序仍需 V6/V7 |
| 比例导引法 | 已进入主链路 | `guidance_png_update` 使用三维 PNG 输出 ECEF 加速度指令 |
| 飞控状态机/调度器 | 已进入主链路 | 模式状态机、调度器配置校验和按 `scheduler.tasks[]` 执行的多速率缓存更新已进入飞控静态库 |
| 自动驾驶仪/控制分配 | 已进入工程主链路 | 自动驾驶仪、舵偏分配、边界单测和标准机动控制品质报告已覆盖；真实型号控制品质仍需 V6/V7 |
| UDP 闭环通信 | 已进入主链路 | 固定小端线协议、CRC、版本和 instance_id 校验已接入 |
| 共享内存通信 | 未使用 | 当前设计选择 UDP；共享内存可作为后续接口扩展 |
| 多实例独立运行 | 已进入主链路 | 管理器读取逐实例配置和显式种子，执行端口预检、应用层 ready 心跳并输出批次摘要 |
| GPU 并行化 | 未使用 | 当前为 CPU 多进程并行；GPU 属于 P8 之后性能扩展 |

对应的设计收敛基线见 [design.md 第 21 节](design.md#21-设计完成基线与落地边界)。
