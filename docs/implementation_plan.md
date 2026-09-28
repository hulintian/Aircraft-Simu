# 飞控与环境闭环仿真系统实现计划

> 本文是阶段实现计划和验收基线，不是当前完成度清单，也不是新成员的第一入口。
> 当前完成度以 `README.md` 和 [project_framework.md](project_framework.md) 为准。
> 上手构建、运行和常见修改路径见 [onboarding.md](onboarding.md)；与成熟飞行仿真软件的差距
> 和后续路线见 [flight_sim_software_comparison.md](flight_sim_software_comparison.md)。
> 统一验收命令和 V0-V7 证据边界见 [verification_guide.md](verification_guide.md)。

## 1. 实现目标

本实现计划对应 `docs/design.md` 中的工业级设计。目标是按可交付阶段逐步实现：

- `common` 共享基础库。
- `environment_sim` 环境仿真程序。
- `flight_control_sim` 飞控模拟程序。
- `tools/instance_manager` 多实例管理器。
- JSON 配置、二进制协议、日志、回放和批量统计。

实现时必须遵守以下约束：

- 核心代码使用 C11。
- 构建系统使用 CMake。
- 一个飞行实例等于一个 `environment_sim` 进程加一个 `flight_control_sim` 进程。
- 实例之间不存在运行时依赖。
- 环境真值主坐标使用 ECEF，配置和地图使用 LLA。
- 配置统一使用 JSON。
- 主循环中不做不可控动态内存分配。
- 所有网络输入、配置输入、浮点输入都必须校验。

## 2. 总体交付顺序

推荐按以下顺序实现：

```text
P0  工程骨架
P1  common 基础库
P2  协议与配置
P3  单实例双进程闭环
P4  地球坐标与地图地形
P5  环境模型与传感器模型
P6  飞控任务、制导与保护
P7  多实例管理器
P8  日志、回放、批量验证
```

不要先写复杂模型再补工程底座。先把构建、协议、配置、日志和测试框架打稳，后续模型才能持续替换。

### 2.1 当前执行状态

截至 2026-08-20，P0-P8 计划内 V0-V5 仓库验收入口均已实现。这里的“完成”只表示代码、
配置、工具和自动测试闭合，不是产品完成百分比，也不包含真实型号数据标定、外部软件
交叉验证或 HIL/实测 V6-V7。多轴成熟度见
[current_progress_and_gaps.md](current_progress_and_gaps.md)。

| 阶段 | 完成证据 |
|---|---|
| P0-P2 | 三个主程序 help/version/退出码、严格构建、公共库/配置/协议单测 |
| P3-P4 | 双进程 LOCKSTEP/FREE_RUNNING、SIL 实时性能、完整 manifest/配置快照、真实来源 DEM 与 LOS 回归 |
| P5-P6 | 6DOF/质量/风/脚本目标/连续命中/完整故障矩阵、内部日志和标准机动控制品质报告 |
| P7 | 并发/串行、继续/停止失败策略、端口预检/ready 心跳和批次摘要集成测试 |
| P8 | 被动/带飞控回放和 Monte Carlo manifest、精确/容差比较、统计、绘图和 4/16/128 压力层 |

## 3. P0 工程骨架

### 3.1 目标

建立可编译、可测试、可扩展的 CMake 工程。

### 3.2 目录

创建：

```text
common/
flight_control_sim/
environment_sim/
tools/instance_manager/
tools/map_preprocess/
tools/replay/
tools/log_convert/
tools/batch_runner/
configs/baseline/
tests/
```

### 3.3 任务

- 顶层 `CMakeLists.txt`。
- 每个子项目一个 `CMakeLists.txt`。
- 设置 C11。
- 打开严格编译警告。
- 添加 `Debug`、`Release` 配置。
- 添加 `CTest`。
- 添加最小空程序：
  - `environment_sim`
  - `flight_control_sim`
  - `instance_manager`

### 3.4 验收

```text
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

三个可执行程序能启动并打印版本、参数帮助和退出码。

## 4. P1 common 基础库

### 4.1 目标

实现全部进程共享的基础能力。

### 4.2 模块顺序

按以下顺序写：

```text
status
math_constants
vec3
matrix3
quaternion
sim_time
random
crc32
ring_buffer
logger
```

### 4.3 关键接口

`status.h`：

```c
typedef enum SimStatus {
    SIM_OK = 0,
    SIM_ERR_INVALID_ARG,
    SIM_ERR_OUT_OF_RANGE,
    SIM_ERR_BAD_PACKET,
    SIM_ERR_TIMEOUT,
    SIM_ERR_CONFIG,
    SIM_ERR_NUMERIC,
    SIM_ERR_INTERNAL
} SimStatus;
```

`vec3.h` 至少提供：

```text
vec3_add
vec3_sub
vec3_scale
vec3_dot
vec3_cross
vec3_norm
vec3_normalize
vec3_isfinite
```

`quaternion.h` 至少提供：

```text
quat_normalize
quat_multiply
quat_to_dcm
quat_integrate
quat_isfinite
```

### 4.4 测试

必须覆盖：

- 点乘、叉乘、范数。
- 四元数归一化。
- DCM 正交性。
- CRC32 稳定性。
- 环形缓冲区延迟读取。
- 随机数固定种子可重复。

## 5. P2 协议与 JSON 配置

### 5.1 目标

实现稳定的二进制协议和 JSON 配置加载。

### 5.2 协议模块

文件：

```text
common/include/common/protocol.h
common/include/common/packet.h
common/src/protocol.c
common/src/packet.c
```

先实现：

- `PacketHeader`。
- `SensorFrame`。
- `ControlCommand`。
- encode。
- decode。
- CRC 校验。
- magic/version/type/instance_id 检查。

### 5.3 配置模块

文件：

```text
common/include/common/config.h
common/src/config.c
```

C 侧使用 JSON 库，但对业务模块只暴露封装接口：

```text
config_load_file
config_get_int
config_get_double
config_get_bool
config_get_string
config_get_array_double
config_require_section
config_validate_schema
```

### 5.4 配置文件

先创建最小可用版本：

```text
configs/baseline/scenario.json
configs/baseline/flight_control.json
configs/baseline/faults.json
configs/baseline/runtime.json
```

### 5.5 验收

- 错误 JSON 必须报错。
- 缺必填字段必须报错。
- 不支持的 `schema_version` 必须报错。
- 包解码遇到错误 `instance_id` 必须拒绝。
- 包 CRC 错误必须拒绝。

## 6. P3 单实例双进程闭环

### 6.1 目标

完成一个 `FlightInstance` 的最小闭环：

```text
environment_sim(instance 0)
  -> SensorFrame
  -> flight_control_sim(instance 0)
  -> ControlCommand
  -> environment_sim(instance 0)
```

此阶段重点是进程、协议、时序和日志，不追求复杂动力学。

### 6.2 environment_sim

先实现：

```text
env_app
env_config
env_context
env_interface_udp
recorder
```

主循环：

```text
load_config
bind_udp
init_context
while running:
    build_sensor_frame
    send_sensor_frame
    receive_control_command
    record_logs
    advance_time
```

### 6.3 flight_control_sim

先实现：

```text
fc_app
fc_config
fc_context
fc_interface_udp
fc_scheduler
command_manager
```

主循环：

```text
load_config
bind_udp
while running:
    receive_sensor_frame
    validate_instance_id
    produce_control_command
    send_control_command
    record_logs
```

### 6.4 验收

- 两个进程可以独立启动。
- `instance_id` 不匹配时双方拒绝数据包。
- `LOCKSTEP` 模式下每帧有一条传感器帧和一条控制帧。
- 生成：
  - `run_manifest.json`
  - 三个实际输入配置字节快照及逐文件 CRC32
  - `sensor_log.bin`
  - `command_log.bin`
  - `event_log.txt`
  - `summary.json`

## 7. P4 地球坐标与地图地形

### 7.1 目标

实现真实地图球形地球环境基础。

### 7.2 模块

```text
earth_model
geo_coordinate
map_tile
terrain_model
```

### 7.3 实现顺序

1. WGS-84 参数。
2. LLA -> ECEF。
3. ECEF -> LLA。
4. ECEF -> ENU/NED。
5. 地形瓦片头解析。
6. DEM 双线性插值。
7. AGL 计算。
8. 地表碰撞判定。
9. LOS 地形遮挡采样接口。

### 7.4 测试

- LLA/ECEF 往返误差。
- ENU/NED 方向正确。
- DEM 插值边界点正确。
- 缺失瓦片策略正确。
- AGL 小于等于零触发地表碰撞。

## 8. P5 环境模型与传感器模型

### 8.1 目标

实现可替换的环境模型链路。

### 8.2 模块顺序

```text
world_state
target_model
gravity_model
atmosphere_model
mass_model
propulsion_model
aero_model
actuator_model
missile_plant_6dof
sensor_noise
sensor_imu
sensor_accel
sensor_speed
sensor_seeker
hit_detect
fault_injection
```

### 8.3 重点

此阶段可以使用占位气动模型和占位推力模型，但接口必须是六自由度接口：

```text
force_b
moment_b
mass
inertia_b
actuator_pos
```

不要把控制指令直接写进位置更新。控制指令必须经过执行机构模型，再进入力/力矩或加速度级接口。

### 8.4 验收

- 真值状态使用 ECEF。
- 日志输出 LLA、ECEF、AGL。
- 传感器输出可加噪声、延迟、丢包。
- 导引头输出 range、LOS unit、LOS rate、closing velocity。
- 故障注入按时间触发。

### 8.5 当前执行窗口

P5 当前已经完成环境力链、6DOF 积分器、脚本目标机动、步间连续命中、四类传感器误差、
采样、延迟、丢包、完整 `faults.json` 故障矩阵、故障统计、错误配置拒绝和固定种子闭环
一致性回归：

```text
faults.json
  -> fault_injection_config
  -> fault_runtime_state
  -> sensor/actuator fault action
  -> SensorFrame fault flags / actuator state
  -> event_log.txt
```

故障事件至少包含：

```text
id
target
start_time_s
duration_s
fault_type
parameters
enabled
```

实现已覆盖第 14 节设计故障矩阵：

- 传感器强制无效。
- 传感器附加偏置。
- 传感器线性漂移。
- 传感器斜坡恢复偏置。
- 传感器确定性噪声增大、起点捕获卡滞、逐测量动态延迟和饱和。
- 传感器卡常值 / 常值保持。
- 传感器整帧丢包窗口。
- 锁步安全的通信窗口故障：`sensor.frame` / `COMMUNICATION_LOSS` 会清除整帧测量有效位并置位 dropout flags。
- 锁步安全的通信帧延迟故障：`sensor.frame` / `COMMUNICATION_DELAY` 或
  `FRAME_DELAY` 会用固定容量延迟线发送滞后 `SensorFrame`，`value` 为 1 到 16 的延迟步数。
- 锁步安全的通信乱序故障：`sensor.frame` / `COMMUNICATION_REORDER` 或
  `FRAME_REORDER` 会在首帧预热后按 LOCKSTEP 发送上一帧，覆盖旧帧拒绝和控制保持路径。
- 通信层 `sensor.frame` 故障支持 `recovery_hold_s` / `recovery_timeout_s`，可在故障窗口结束后
  继续保持故障效果再恢复，用于覆盖飞控超时后的恢复边界。
- 执行机构卡死。
- 执行机构偏置、比例缩放、速率/行程退化、命令延迟和失能。
- 实际 UDP 报文丢弃、重复和 CRC 损坏；FREE_RUNNING 闭环覆盖损坏报文拒绝和后续恢复。

当前基础验收已经证明故障触发后：

- `event_log.txt` 记录触发和恢复。
- `sensor_fault_flags` 或执行机构状态发生预期变化。
- 飞控在导引头无效时仍按 LOCKSTEP 返回受控零指令。
- 同一随机种子下两次运行结果一致。
- `summary.json` 或 `campaign_summary.json` 能汇总故障触发次数和影响范围。

P5 工程验收后续已补齐：

- `burst_period_s` / `burst_active_s` 周期突发通信窗口。
- `COMMUNICATION_JITTER` 固定模式变延迟和恢复跳变测试。
- 珠峰附近真实来源 DEM fixture、来源/哈希清单、预处理 LOS 和双进程遮挡闭环。
- 完整质心/惯量张量演化，以及风切变、阵风和固定种子湍流模型。

`FREE_RUNNING` 已用于基础非锁步恢复；每传感器独立时间戳、时钟漂移和操作系统网络栈
时序相关性仍属于协议 v2 与外部网络验证范围。

## 9. P6 飞控任务、制导与保护

### 9.1 目标

实现飞控模拟程序的工程结构。

### 9.2 模块顺序

```text
fc_modes
fc_health
fc_scheduler
navigation
estimator
guidance_png
guidance_manager
autopilot
command_manager
safety_monitor
```

### 9.3 三维比例导引

实现：

$$
\mathbf a_c
=
N V_c
\left(
\boldsymbol\omega_{\text{LOS}}
\times
\hat{\mathbf r}
\right)
$$

然后执行：

- 范数限幅。
- 变化率限制。
- NaN/Inf 检查。
- 旧帧拒绝。
- 传感器超时保护。
- 目标测量无效保护。

### 9.4 状态机

必须实现：

```text
FC_POWER_ON
FC_SELF_TEST
FC_WAIT_SENSOR
FC_NAV_READY
FC_GUIDANCE_STANDBY
FC_GUIDANCE_ACTIVE
FC_COMMAND_HOLD
FC_DEGRADED
FC_FAULT
FC_SHUTDOWN
```

### 9.5 验收

- 固定传感器输入下制导输出可复现。
- LOS rate 方向测试通过。
- 闭合速度异常时不输出非受控大指令。
- 指令限幅和变化率限制生效。
- 飞控内部日志记录每次保护动作。

### 9.6 当前执行窗口

P6 当前已经把飞控主链路从单一 PNG 函数扩展为可测试静态库：

```text
SensorFrame
  -> safety_monitor
  -> estimator/navigation
  -> fc_modes/fc_health
  -> guidance_manager/guidance_png
  -> autopilot
  -> command_manager
  -> ControlCommand
```

第一批实现已经完成：

- `missile_flight_control` 静态库和 `flight_control_tests`。
- `FC_POWER_ON` 到 `FC_SHUTDOWN` 的模式状态机。
- 健康/保护位写入 `ControlCommand.command_status`。
- 导航估计、制导管理、虚拟自动驾驶仪和命令管理模块。
- 姿态/角速度自动驾驶仪、俯仰/偏航舵偏控制分配，并由环境气动模型消费。
- 严格按 `scheduler.tasks[]` 执行的多速率导航、制导和控制缓存更新。
- 加速度幅值限制和变化率限制。
- 旧帧、NaN/Inf、传感器超时、目标测量无效和闭合速度异常保护。
- 闭环测试会解码 `command_log.bin`，验证导引头预热时进入命令保持，
  有效测量后进入制导激活并触发变化率限制。

P6 工程验收后续已补齐：

- `fc_internal_log.bin` 固定记录格式和 `log_convert --type fc-internal` 正式解码。
- 使用真实 `FlightController` 链的阶跃、指令反向和丢包恢复标准机动。
- 上升时间、调节时间、超调、稳态误差、最大变化率和饱和占比的版本化阈值报告。
- 命令保持超时按最后一帧新鲜命令计时，并有恢复边界单测。

真实型号带宽、稳定裕度和飞行品质仍需 V6/V7 数据与试验，不作为 SIL 工程 P6 完成条件。

## 10. P7 多实例管理器

### 10.1 目标

实现多个互不依赖的 `FlightInstance`。

### 10.2 模块

```text
tools/instance_manager
  manager_config
  instance_plan
  port_allocator
  process_launcher
  process_monitor
  summary_collector
```

### 10.3 规则

- 一个实例只包含一对进程。
- 实例之间不通信。
- 实例之间不共享运行时状态。
- 单个实例失败不影响其他实例。
- 只允许共享只读静态资源。
- 每个实例必须有独立日志目录。

### 10.4 验收

- `PARALLEL` 模式可以同时运行多个实例。
- `SEQUENTIAL` 模式可以串行运行多个实例。
- 端口无冲突。
- `instance_id` 唯一。
- 单个实例失败时，其他实例继续运行。
- 生成 `campaign_summary.json`。

### 10.5 当前执行窗口

P7 当前已经把 `instance_manager` 从固定 baseline 启动器扩展为逐实例计划执行器：

```text
runtime.instances[]
  -> InstancePlan(instance_id, scenario, flight_control, faults, random_seed, enabled)
  -> tools(environment_program, flight_control_program)
  -> port preflight
  -> flight_control_sim --ready-port
  -> PACKET_HEARTBEAT ready
  -> environment_sim --random-seed
  -> flight_control_sim --config
  -> campaign_summary.json
```

第一批实现已经完成：

- 读取启用的 `instances[]`，传递逐实例场景、飞控、故障配置和显式随机种子。
- 环境程序支持 `--random-seed`，并在 `run_manifest.json` 记录实际种子。
- 支持 `PARALLEL`、`SEQUENTIAL`、并发上限和基础失败策略。
- 支持通过 `runtime.tools.environment_program` 和 `runtime.tools.flight_control_program`
  配置子程序路径，管理器测试会从 `/tmp` 启动以验证不依赖仓库根目录。
- 启动前校验 `instance_id` 唯一、端口唯一和 UDP 端口可绑定。
- 飞控启动完成后发送应用层 ready 心跳，管理器收到合法 `PACKET_HEARTBEAT`
  后才启动环境程序。
- 管理器子进程失败时返回非零退出码，不再把失败批次伪装为成功。
- `campaign_summary.json` 汇总 campaign id、命中/未命中/超时、脱靶量 min/max/mean/std、
  失败实例，以及每个实例的端口、配置路径、随机种子、退出状态和故障统计。
- `instance_manager_test` 覆盖两实例并发和串行计划、端口隔离、端口占用预检、显式种子、
  应用层 ready 心跳、飞控早退、`CONTINUE_ON_FAILURE` 后续继续、
  `STOP_ON_FAILURE` 跳过路径和批次摘要。

P7 计划内主链路能力已完成；后续只保留更复杂的调度器集成，例如远程节点启动和资源配额。

## 11. P8 日志、回放、批量验证

### 11.1 目标

让系统可以复现、回放、统计和回归。

### 11.2 工具

```text
tools/replay
tools/log_convert
tools/compare_logs
tools/batch_stats
tools/batch_runner
tools/plot
```

### 11.3 功能

- 二进制日志转 CSV。
- 回放 `sensor_log.bin` 驱动飞控。
- 对比两次 `ControlCommand`。
- 对比两次真值或控制日志，定位首个发散帧。
- 汇总多个实例的 `summary.json` 和 `campaign_summary.json`。
- 输出命中率、脱靶量均值、标准差、失败实例列表。
- 输出故障影响、气动包线越界、数值诊断异常和失败原因分布。

### 11.4 验收

- 固定随机种子结果可复现。
- 相同输入日志重新驱动飞控，输出一致。
- 回归误差超过阈值时测试失败。
- 截断日志、CRC 错误、实例号不匹配和帧数不一致必须失败并给出明确原因。
- 回归支持两类基准：
  - 逐字节完全一致基准。
  - 浮点绝对/相对容差基准。
- `trajectory_diagnostics.csv` 或等价诊断输出覆盖四元数范数、DCM 正交性、
  质量/惯量有效性、气动模型包线状态和积分器类型。

### 11.5 当前执行窗口

P8 当前已经完成第一批可验证工具和诊断输出：

- 新增 `tools/log_convert`，支持 `--type sensor|command|fc-internal`、`--instance-id`、
  `--input`、`--output` 和 `--manifest`，并写 `REPLAY_PASSIVE` sidecar manifest。
- `sensor_log.bin` 与 `command_log.bin` 可按固定线格式解码、校验实例号和 CRC，
  并转换为 CSV。
- 新增 `tools/replay`，支持使用 `sensor_log.bin` 离线驱动飞控并生成新的
  `command_log.bin`，同时写 `REPLAY_WITH_FC` manifest 和输入/配置/输出 CRC。
- 新增 `tools/compare_logs`，支持比较 `sensor_log.bin`、`command_log.bin` 或
  `trajectory.csv`，可输出首个发散帧、最大误差和 `EXACT`/`TOLERANCE` 模式 JSON；
  `configs/verification/log_compare_exact.json` 和 `log_compare_tolerance.json`
  分别提供逐字节/逐行精确基准与版本化绝对/相对容差。
- 新增 `tools/batch_stats`，支持汇总多个 `summary.json` 或
  `campaign_summary.json`，按全部实例输出命中率、脱靶量样本数/均值/标准差、
  失败实例、失败原因分布、故障影响统计和
  数值诊断最大/最小值、模型降级 flags 按位或、气动 flags 按位或、气动外推采样数和
  wall-clock 性能统计。
- 新增 `tools/batch_runner`，支持按清单顺序调用 `instance_manager --runtime`，
  并可选调用 `batch_stats` 汇总清单中的统计输入；同时支持从 runtime 模板
  确定性生成 Monte Carlo 清单，展开 `${sample_index}`、`${random_seed}` 和
  `${sample_output_dir}` 占位符，并支持 `${uniform:stream:min:max}` 确定性均匀扰动和
  `${lhs_uniform:stream:min:max}` 确定性 LHS 均匀分层扰动、
  `${halton_uniform:base:min:max}` 确定性 Halton 低差异均匀扰动、
  `${normal:stream:mean:stddev}` 确定性正态扰动、
  `${lognormal:stream:mu:sigma}` 确定性对数正态扰动、
  `${truncated_normal:stream:mean:stddev:min:max}` 确定性截断正态扰动、
  `${choice:stream:option|option}` 确定性离散选择，以及
  `${correlated_normal:stream:base_stream:mean:stddev:rho}` 确定性相关正态扰动。
  手写清单和模板生成两种路径均写 `MONTE_CARLO` 工作流 manifest。
- `closed_loop_test` 已接入真实 `sensor_log.bin` 回放和 `command_log.bin`
  比较，验证固定输入日志可重放出一致控制输出。
- 顶层 CMake 提供 `MISSILE_ENABLE_COVERAGE`、`MISSILE_ENABLE_SANITIZERS`、
  `MISSILE_ENABLE_MEDIUM_TESTS` 和 `MISSILE_ENABLE_LONG_TESTS`；默认有 11 个 CTest，
  sanitizer 构建会在配置阶段检查 ASan/UBSan 运行库。
- 环境侧新增 `trajectory_diagnostics.csv`，记录四元数、DCM、质量/惯量、
  模型降级 flags、气动状态、积分器和合力/力矩诊断；`summary.json`、`campaign_summary.json`
  和 `batch_stats` 会聚合四元数范数误差、DCM 正交性误差、最小质量、最小惯量、
  模型降级 flags 按位或、气动 flags 按位或和气动外推采样数。
- 飞控侧新增固定记录格式 `fc_internal_log.bin`，记录每帧模式、状态和主要控制中间量，
  并由 `log_convert --type fc-internal` 解码。
- 新增 `aero_database` 气动表模块，覆盖版本化固定小端文件、样本 CRC、单位校验、
  单调性校验、样本插值、包络外错误、钳制告警和保持上一有效值策略。
- 环境程序支持 `aerodynamics.table_path` 和
  `aerodynamics.table_extrapolation_policy`，可选气动表已接入统一环境力模型。
- 环境程序支持 `aerodynamics.table_v2_path`，六维 v2 表已由双进程闭环测试覆盖。
- `run_manifest.json` 会记录软件/Git/编译器/构建时间、配置文件列表/CRC32/输入快照、
  运行模式和标准日志路径；同时记录气动表启用状态、表文件路径、内部文件格式版本、
  包络外策略来源/覆盖值、surrogate 模型路径/模型版本/训练数据版本，
  以及地形启用状态、LOS 遮挡开关、缺瓦片策略和瓦片路径列表。
- 新增 `log_convert_test`，覆盖 `command_log.bin` 到 CSV 的转换路径。
- 新增 `p8_tools_test`，覆盖 `replay`、`compare_logs`、`batch_stats`、诊断统计聚合和截断日志失败路径。
- 新增 `batch_runner_test`，覆盖清单运行入口和 Monte Carlo runtime 模板展开。
- 新增 `control_quality_report` 和版本化 criteria，覆盖标准机动与工程阈值。
- 压力测试分为默认 short 4 实例/并发 2、可选 medium 16/4 和 long 128/8，
  均校验 `completed_count`/`failed_count`、随机种子、诊断和 wall-clock 字段。
- 真实来源 DEM fixture、资源清单和真实 DEM LOS 双进程闭环已进入回归。

P8 工程验收已经完成。真实目标机器长时容量、真实气动/器件数据和复杂代理模型训练
属于 V6/V7 扩展，不计入 P8 软件工具链完成度。

### 11.6 P8 后模型保真度扩展

P8 完成后再推进以下能力，避免在回放和回归基础不稳时引入高复杂度模型：

```text
aero_database
  已完成 Mach / AoA / beta 样本插值、版本化文件格式、CRC/单位校验和配置接入；
  v2 已完成 Mach / AoA / beta / 高度 / 俯仰舵偏 / 偏航舵偏六维规则网格、
  64 角多线性插值、固定小端文件、CRC、三种包络策略、主力模型和闭环接入；
  `run_manifest.json` 记录 v2 路径、版本和六轴维度。真实数据基准仍需 V6。

aero_surrogate
  已在 `run_manifest.json` 追踪 surrogate 模型路径、模型版本、训练数据版本和适用包线；
  已接入固定文本格式线性只读推理，覆盖缺项拒绝、Mach/alpha/beta 线性推理、
  包线外拒绝和气动力主路径；
  待扩展离线训练、复杂代理模型和真实数据基准。

map_preprocess
  已完成裸 ASCII 高程网格、ESRI ASCII Grid 和 SRTM HGT 到内部瓦片格式的最小预处理、
  CRC 生成、二进制空间索引输出和加载测试；SRTM HGT 会从 `N30E120.hgt` 这类文件名推导
  1 度瓦片边界，按 big-endian int16 读取高程，并要求对 void 值显式 `--nodata-fill`；
  `map.tile_path` / `map.tile_paths[]` / `map.tile_index_path` 文本/二进制空间索引加载链已进入闭环测试，
  索引内相对瓦片路径会按索引文件所在目录解析；
  `--nodata-fill` 显式缺测填补、`map.terrain.cache_tile_count` 固定槽位 LRU 懒加载/淘汰、
  `summary.json` 地形缓存统计已接入；
  已接入带来源/哈希清单的珠峰附近真实来源 DEM fixture，以及真实 DEM LOS 遮挡闭环场景。

diagnostics
  已聚合四元数范数误差、DCM 正交性误差、最小质量、最小惯量、
  模型降级 flags 按位或、气动 flags 按位或和气动外推采样数。
```

验收要求：

- 气动表文件缺失、单位错误、NaN、非单调网格和 CRC 错误必须拒绝。
- 包线外输入必须按 `ERROR`、`CLAMP_AND_WARN` 或 `HOLD_LAST_VALID` 策略处理。
- 气动表必须在 `run_manifest.json` 中记录资源路径、文件格式版本和策略来源；
  代理模型必须记录模型版本、训练数据版本和适用包线。
- 真实 DEM 场景必须覆盖缺瓦片错误路径、平坦填充路径和 LOS 遮挡闭环路径。

## 12. 开发提交建议

建议按小提交推进：

```text
commit 1: cmake skeleton
commit 2: common status/vec3/math tests
commit 3: protocol encode/decode tests
commit 4: json config loader
commit 5: minimal environment_sim
commit 6: minimal flight_control_sim
commit 7: udp lockstep loop
commit 8: logs and manifest
commit 9: geo coordinate
commit 10: terrain model
commit 11: seeker sensor
commit 12: png guidance
commit 13: safety monitor
commit 14: instance manager
commit 15: campaign summary
```

每个提交都必须可编译，不能把大面积半成品堆在一个提交里。

## 13. 优先级

### 13.1 必须先做

```text
CMake
common
protocol
config
single-instance UDP loop
logging
```

这些是后续全部模块的基础。

### 13.2 第二优先级

```text
ECEF/LLA
terrain
sensor models
flight control scheduler
PNG guidance
safety monitor
```

### 13.3 第三优先级

```text
multi-instance manager
fault injection
replay
batch statistics
plot tools
```

## 14. 风险控制

| 风险 | 处理 |
|---|---|
| 一开始模型太复杂导致闭环跑不起来 | 先打通协议和主循环，再填模型 |
| 多实例端口冲突 | 使用统一端口分配器 |
| 实例之间出现隐式依赖 | 禁止共享运行时状态，只允许只读静态资源 |
| JSON 配置字段不一致 | 加 schema_version 和配置校验 |
| ECEF/LLA 坐标符号错误 | 做坐标转换单元测试 |
| 制导方向错误 | 固定几何场景测试 LOS rate 和加速度方向 |
| 日志太大 | 二进制日志加可配置采样和 flush 策略 |
| 单实例失败影响批次 | 默认 CONTINUE_ON_FAILURE |

## 15. 当前最小可执行目标

第一轮编码的最小目标：

```text
1. 能编译。
2. 能启动一个 environment_sim。
3. 能启动一个 flight_control_sim。
4. 双方通过 UDP 交换带 instance_id 的二进制包。
5. 能跑固定步长 LOCKSTEP。
6. 能生成 run_manifest、sensor_log、command_log、summary。
7. instance_id 不匹配时拒绝数据包。
```

完成这个目标后，再进入地球坐标、地图、传感器和制导模型实现。
