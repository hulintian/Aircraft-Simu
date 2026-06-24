# 飞控与环境闭环仿真系统实现计划

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

P5 当前已经完成环境力链、6DOF 积分器、四类传感器误差、采样、延迟、丢包、
基础 `faults.json` 故障注入、故障统计、错误配置拒绝和固定种子闭环一致性回归：

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

第一批实现已经覆盖仿真正确性需要的基础故障类型：

- 传感器强制无效。
- 传感器附加偏置。
- 传感器整帧丢包窗口。
- 执行机构卡死。
- 执行机构比例缩放。

当前基础验收已经证明故障触发后：

- `event_log.txt` 记录触发和恢复。
- `sensor_fault_flags` 或执行机构状态发生预期变化。
- 飞控在导引头无效时仍按 LOCKSTEP 返回受控零指令。
- 同一随机种子下两次运行结果一致。
- `summary.json` 或 `campaign_summary.json` 能汇总故障触发次数和影响范围。

后续扩展不再阻塞 P6，但仍应补充：

- 更多真实故障类型，例如卡常值、漂移阶跃、恢复斜坡和通信窗口丢包。
- 真实 DEM 数据集加载后的 LOS 遮挡闭环场景。

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

## 11. P8 日志、回放、批量验证

### 11.1 目标

让系统可以复现、回放、统计和回归。

### 11.2 工具

```text
tools/replay
tools/log_convert
tools/batch_runner
tools/plot
```

### 11.3 功能

- 二进制日志转 CSV。
- 回放 `sensor_log.bin` 驱动飞控。
- 对比两次 `ControlCommand`。
- 汇总多个实例的 `summary.json`。
- 输出命中率、脱靶量均值、标准差、失败实例列表。

### 11.4 验收

- 固定随机种子结果可复现。
- 相同输入日志重新驱动飞控，输出一致。
- 回归误差超过阈值时测试失败。

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
