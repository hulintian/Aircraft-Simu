# 项目框架与当前实现

> 更新日期：2026-06-18
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
        missile_environment                    guidance_png
        地球/地形/动力学模型                           │
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
| `environment_sim` | 可执行程序 | 环境进程 |
| `flight_control_sim` | 可执行程序 | 飞控进程 |
| `instance_manager` | 可执行程序 | 多实例编排工具 |
| `common_tests` | 测试程序 | 公共库单元测试 |
| `environment_tests` | 测试程序 | 环境模型单元测试 |
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
  src/                   当前飞控主循环与三维 PNG
  tests/                 预留，当前为空

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
  6. 使用三维比例导引计算 ECEF 加速度指令
  7. 编码 ControlCommand 并通过 UDP 返回
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

飞控指令仍是加速度级虚拟接口，气动控制面偏角当前为零。在 P6 自动驾驶仪和
控制分配完成前，该链路用于验证环境物理项和 6DOF 软件结构，不代表真实舵面闭环。

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

环境程序使用 `campaign.base_random_seed + instance_id` 作为实例种子，再为各
传感器派生独立随机流。飞控检查 `sensor_valid_flags`；导引头延迟预热、丢包或
地形遮挡时仍按 LOCKSTEP 返回受控零指令。`faults.json` 尚未接入。

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

当前已进入执行链的飞控逻辑较小：

```text
UDP SensorFrame
  -> 协议/实例号校验
  -> 旧序列帧拒绝
  -> 导引头有效位检查
  -> guidance_png_update
  -> 加速度幅值限幅
  -> UDP ControlCommand
```

已经实现：

- 三维比例导引。
- 输入有限值和距离范围检查。
- 加速度幅值限制。
- 旧序列帧拒绝。
- 无效导引头测量的受控零指令响应。

尚未接入或仅有接口定义：

- 多速率调度器。
- 飞控状态机和模式转换。
- 导航解算与状态估计。
- 制导管理器。
- 自动驾驶仪和控制分配。
- 命令保持、降级和安全监视。
- 加速度指令变化率限制；配置已读取，但算法未使用该参数。

`flight_control.json` 中的 `scheduler` 和大部分 `safety` 配置目前仅用于结构
预留或必填节校验，不会驱动实际任务调度和保护逻辑。

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
| `flight_control.json` | PNG 参数；调度和安全配置部分预留 |
| `runtime.json` | 端口、输出目录、实例数和并发上限 |
| `faults.json` | 故障脚本示例；当前未被运行程序读取 |

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
- 按 `max_parallel_instances` 限制并发实例对数量。
- 启动和回收环境、飞控子进程。
- 写出 `campaign_summary.json`。

当前限制：

- 使用固定的 `configs/baseline/scenario.json` 和
  `configs/baseline/flight_control.json`。
- 未读取 `instances[]` 中的逐实例配置、随机种子和故障文件。
- 未使用 `schedule`、`failure_strategy` 等运行策略。
- 子程序路径固定为 `./build/...`，依赖从仓库根目录启动。
- 启动同步使用固定一秒等待，没有就绪握手。
- 管理器生成失败摘要后仍固定返回成功退出码。

环境进程会按 `base_random_seed + instance_id` 派生不同随机流，因此已经可以
产生可重复的实例差异。但管理器尚未使用 `instances[]` 中的显式种子和逐实例
配置，所以仍不是完整的 Monte Carlo 执行框架。

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

当前 CTest 有三个测试：

| 测试 | 覆盖范围 |
|---|---|
| `common_tests` | 数学、四元数、随机数、环形缓冲区、配置和协议 |
| `environment_tests` | 坐标、地形瓦片、LOS、执行机构、环境模型、6DOF 和噪声 |
| `closed_loop_test` | 双进程 UDP 锁步、命中结果和主要运行产物 |

当前主要测试缺口：

- 没有飞控 PNG、状态机或保护逻辑的独立单元测试。
- 没有实例管理器自动化测试。
- 没有故障注入、丢包、超时恢复和错误序列集成测试。
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
| P5 环境与传感器 | 80% | 环境力链和四类传感器误差/延迟/丢包已集成；故障脚本未接入 |
| P6 飞控系统 | 30% | PNG 可用，其余飞控框架多数为占位 |
| P7 多实例 | 55% | 并发进程管理可用，逐实例配置和策略未实现 |
| P8 回放与验证 | 15% | 日志和绘图可用，回放和批量验证工具为空 |

以完整规划为目标，当前工程功能完成度约为 **72%**；以“可构建、可运行、
可命中的最小双进程闭环”为目标，核心 MVP 已完成。

## 12. 建议的后续顺序

1. 解析 `faults.json`，按仿真时间注入传感器和执行机构故障并记录事件。
2. 为飞控增加独立静态库和单元测试，再实现调度器、状态机、估计器和保护。
3. 让 `instance_manager` 读取 `instances[]`，传递逐实例配置和随机种子。
4. 实现二进制日志回放、日志转换、地图预处理和批量统计工具。
5. 增加 sanitizer、覆盖率、异常通信和多实例回归测试。
