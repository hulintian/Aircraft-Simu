# 飞控与环境闭环仿真系统

本项目使用 C11 和 CMake 实现纯软件飞控闭环仿真。一个飞行实例由两个互相独立的进程组成：

```text
environment_sim
    -> SensorFrame/UDP
flight_control_sim
    -> ControlCommand/UDP
environment_sim
```

多个实例只共享只读程序和配置文件，不共享运行时状态。每个实例使用独立端口、独立进程和独立输出目录。

文档入口：

- [项目框架与当前实现](docs/project_framework.md)：以现有源码为准的模块、数据流、集成状态和缺口。
- [总体设计](docs/design.md)：系统的目标架构和完整能力设计。
- [实现计划](docs/implementation_plan.md)：P0-P8 阶段任务和验收标准。

## 当前进度

实现工作按照 [实现计划](docs/implementation_plan.md) 推进。

当前总体实现进度约为 **72%**。该比例按实现计划中的模块和验收项粗略计算，
表示工程功能完成度，不表示已经达到可交付武器级或适航级软件成熟度。

| 阶段 | 状态 | 估算 | 已实现内容 |
|---|---|---:|---|
| P0 工程骨架 | 完成 | 100% | C11、CMake、严格警告、CTest、三个可执行程序 |
| P1 common | 完成 | 100% | 状态码、向量、矩阵、四元数、仿真时间、随机数、CRC32、环形缓冲区、日志 |
| P2 协议与配置 | 完成 | 100% | JSON 语法、schema_version 和必填字段校验；固定小端线协议、CRC、版本和实例号校验 |
| P3 单实例闭环 | 完成 | 100% | 双进程 UDP LOCKSTEP、二进制日志、运行清单、事件日志、摘要和集成测试 |
| P4 地球与地形 | 基本完成 | 90% | WGS-84、LLA/ECEF、ENU/NED、固定格式地形瓦片、DEM 插值、AGL、碰撞和 LOS 遮挡；真实瓦片加载链未接入 |
| P5 环境与传感器 | 进行中 | 80% | 环境力链及 IMU、加速度计、速度计、导引头的噪声、采样、延迟和丢包已接入 |
| P6 飞控系统 | 进行中 | 30% | 三维比例导引、旧帧拒绝、NaN/Inf 检查和加速度幅值限制 |
| P7 多实例 | 进行中 | 55% | 实例对启动、并发上限、端口和输出隔离、任务摘要 |
| P8 回放与验证 | 起步 | 15% | 二进制日志、轨迹 CSV、任务摘要和绘图脚本；回放工具未实现 |

当前环境主循环已经使用 ECEF 真值状态、六自由度刚体状态、三轴虚拟执行机构和
RK4 积分。重力、大气、气动、推进、质量消耗和地球自转项已经通过统一环境力模型
组成 `force_b`、`moment_b` 和 ECEF 重力输入。基线启用重力、大气、低阻力气动和
地球自转，推进模型进入主链路但默认关闭。

飞控指令当前仍通过加速度级虚拟执行机构形成等效控制力；在自动驾驶仪和控制分配
完成前，不把该接口描述为真实舵面闭环。四类传感器使用实例私有确定性随机流，
支持偏置、白噪声、随机游走、量化、限幅、采样保持、延迟和整帧丢包。基线导引头
延迟为 20 ms，飞控会对延迟预热、丢包或遮挡帧返回受控零指令。

故障脚本注入、飞控状态机、调度器、估计器、自动驾驶仪、变化率限制和回放工具
仍未完成。地形接口已经完成，但仓库中尚无真实 DEM 数据集，基线配置使用
`FLAT_FILL` 零高程策略。

## 项目 Skill

仓库包含项目级 Codex Skill：

```text
.codex/skills/missile-sim-development/
```

该 Skill 固化项目事实读取顺序、架构约束、P0-P8 推进策略、注释要求和闭环验收流程。
调用示例：

```text
$missile-sim-development 核对当前进度并继续实现下一项计划
```

## 目录结构

```text
.codex/skills/          项目级 Codex 开发 Skill
common/                 公共数学、配置、协议和日志库
environment_sim/        环境、地球、地图和飞行器模型
flight_control_sim/     导航、制导、控制和飞控保护
tools/instance_manager/ 多实例进程编排
configs/baseline/       基线 JSON 配置
docs/                   当前框架、目标设计和实现计划
scripts/                轨迹绘图辅助脚本
tests/                  跨进程闭环测试
runs/                   仿真输出，默认不作为源代码输入
```

## 构建

要求 Linux、CMake 3.16 及以上版本，以及支持 C11 的编译器。

```sh
CCACHE_DISABLE=1 cmake -S . -B build
CCACHE_DISABLE=1 cmake --build build
```

运行测试：

```sh
CCACHE_DISABLE=1 ctest --test-dir build --output-on-failure
```

当前 CTest 包含：

- `common_tests`：数学、随机数、环形缓冲区、配置和二进制协议。
- `environment_tests`：地球坐标、地形、执行机构、环境模型和 6DOF 积分。
- `closed_loop_test`：飞控与环境双进程 UDP 闭环回归。

`closed_loop_test` 会在本机回环地址创建 UDP 端口。如果运行环境限制网络命名空间，需要允许本地 UDP 回环通信。

## 单实例运行

先启动飞控，再启动环境程序：

```sh
./build/flight_control_sim/flight_control_sim --instance-id 0 &
fc_pid=$!
sleep 0.2
./build/environment_sim/environment_sim --instance-id 0
wait "$fc_pid"
```

也可以分别指定配置：

```sh
./build/flight_control_sim/flight_control_sim \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --runtime configs/baseline/runtime.json

./build/environment_sim/environment_sim \
  --instance-id 0 \
  --scenario configs/baseline/scenario.json \
  --runtime configs/baseline/runtime.json
```

## 多实例运行

基线 [runtime.json](configs/baseline/runtime.json) 默认定义 2 个开发测试实例，当前并发上限为 2：

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json
```

当前管理器能够隔离进程、端口和输出目录。环境程序暂时使用
`campaign.base_random_seed + instance_id` 生成实例随机种子，因此不同实例的
传感器结果已可确定性区分；管理器仍未读取 `instances[]` 中各自的场景、飞控、
故障文件和显式随机种子，所以目前只是受限 Monte Carlo 能力。

## 输出文件

默认输出位于：

```text
runs/baseline_dev_001/
  campaign_summary.json
  instance_0000/
    run_manifest.json
    event_log.txt
    sensor_log.bin
    command_log.bin
    trajectory.csv
    summary.json
```

- `run_manifest.json`：软件版本、协议版本、配置路径、端口、步长和实例随机种子。
- `event_log.txt`：启动、命中、通信异常和停止事件。
- `sensor_log.bin`：固定小端线格式的完整传感器报文。
- `command_log.bin`：固定小端线格式的完整控制指令报文。
- `trajectory.csv`：导弹和目标的 ECEF/LLA 状态、AGL、速度、加速度、质量、推进剂、合力、合力矩、距离和闭合速度。
- `summary.json`：命中结果、最小距离、最近点时刻和退出原因。

## 下一阶段

当前开发重点是完成 P5 主链路：

1. 解析 `faults.json` 并按仿真时间触发传感器和执行机构故障。
2. 将故障动作和恢复事件写入 `event_log.txt`。
3. 增加故障、丢包和失效测量的闭环回归。

P5 完成后再集中实现 P6 飞控状态机、调度器、导航估计、自动驾驶仪和完整安全保护。

## 查看轨迹

静态或动画三维轨迹：

```sh
python3 scripts/plot.py
```

程序提示输入文件时填写：

```text
runs/baseline_dev_001/instance_0000/trajectory.csv
```

Plotly 动画：

```sh
python3 scripts/ploty_plot.py
```

带地球球面的 ECEF 轨迹：

```sh
cd scripts
python3 earth_plot.py
```

绘图脚本需要 Python，以及对应脚本使用的 `pandas`、`numpy`、`matplotlib` 或 `plotly`。

## 配置文件

- [scenario.json](configs/baseline/scenario.json)：仿真步长、初始状态、地球、地图和环境模型配置。
- [flight_control.json](configs/baseline/flight_control.json)：飞控任务、比例导引和安全参数。
- [runtime.json](configs/baseline/runtime.json)：实例数量、端口、输出目录和逐实例计划。
- [faults.json](configs/baseline/faults.json)：按仿真时间触发的故障脚本。

所有配置必须是完整合法 JSON，并包含受支持的 `schema_version`。启动程序会拒绝错误 JSON、缺失当前程序要求的必填节和不支持的配置版本。

## 代码规范

- 公共接口和数据结构使用 Doxygen 风格文档注释。
- 注释说明坐标系、单位、所有权、错误条件和数值假设，不重复代码表面含义。
- 仿真主循环不进行不可控动态内存分配。
- 网络、配置和浮点输入必须校验。
- 实例之间禁止共享可变运行时状态。
- 每个阶段必须通过单元测试和双进程闭环回归后再继续扩展。
