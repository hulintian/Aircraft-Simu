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

- [新成员上手指南](docs/onboarding.md)：按阅读顺序、构建运行、工具用途和常见改法快速进入项目。
- [工具使用指南](docs/tools.md)：说明 `instance_manager`、日志转换、回放、比较、批量统计、地图预处理、批跑和绘图工具的用法。
- [项目框架与当前实现](docs/project_framework.md)：以现有源码为准的模块、数据流、集成状态和缺口。
- [总体设计](docs/design.md)：系统的目标架构、数学模型、完整能力设计和当前设计收敛基线。
- [实现计划](docs/implementation_plan.md)：P0-P8 阶段任务和验收标准。
- [与主流飞行仿真软件的差距调研](docs/flight_sim_software_comparison.md)：对比 JSBSim、FlightGear、X-Plane、MSFS、Aerospace Blockset、NASA Trick 和 STK，明确本项目边界与后续路线。
- [飞行仿真与气动弹道研究调研](docs/research_survey_flight_sim_aero_ballistics.md)：历史研究背景，进度数字以 README 和项目框架为准。

## 当前进度

实现工作按照 [实现计划](docs/implementation_plan.md) 推进。

当前总体实现进度约为 **99%**。该比例按实现计划中的模块和验收项粗略计算，
表示工程功能完成度，不表示已经达到可交付武器级或适航级软件成熟度。

| 阶段 | 状态 | 估算 | 已实现内容 |
|---|---|---:|---|
| P0 工程骨架 | 完成 | 100% | C11、CMake、严格警告、CTest、三个可执行程序 |
| P1 common | 完成 | 100% | 状态码、向量、矩阵、四元数、仿真时间、随机数、CRC32、环形缓冲区、日志 |
| P2 协议与配置 | 完成 | 100% | JSON 语法、schema_version 和必填字段校验；固定小端线协议、CRC、版本和实例号校验 |
| P3 单实例闭环 | 完成 | 100% | 双进程 UDP LOCKSTEP、二进制日志、运行清单、事件日志、摘要和集成测试 |
| P4 地球与地形 | 基本完成 | 99% | WGS-84、LLA/ECEF、ENU/NED、固定格式地形瓦片、`map.tile_path`/`map.tile_paths[]`/`map.tile_index_path` 文本/二进制空间索引加载、`cache_tile_count` 固定槽位 LRU 懒加载/淘汰、裸 ASCII/ESRI ASCII Grid/SRTM HGT 预处理、NODATA 显式填补、DEM 插值、AGL、碰撞、LOS 遮挡和合成山脊遮挡闭环；真实 DEM 数据集基准和磁盘缓存未接入 |
| P5 环境与传感器 | 基本完成 | 99% | 环境力链、四类传感器误差/延迟/丢包、基础故障脚本、漂移、斜坡恢复偏置、卡常值、整帧通信丢失、锁步安全通信延迟和上一帧重放乱序故障、故障统计、数值诊断、错误路径和固定种子回归已接入 |
| P6 飞控系统 | 主链路基本完成 | 94% | `missile_flight_control` 静态库、状态机、健康/保护、估计、制导管理、姿态/角速度自动驾驶仪、舵面控制分配、命令管理、多速率调度、幅值/变化率限制、飞控内部日志和单元测试；已覆盖 PNG 方向/限幅/NaN 拒绝、自动驾驶仪内环开关路径、通信恢复后的安全监控恢复边界和连续帧控制品质边界 |
| P7 多实例 | 完成 | 100% | `instances[]` 逐实例计划、显式随机种子、并发/串行调度、子程序路径配置化、端口预检、应用层 ready 心跳、失败返回码、任务摘要和管理器集成测试 |
| P8 回放与验证 | 基本完成 | 99% | 二进制日志、轨迹 CSV、数值诊断、模型降级 flags、气动包线 flags 和 wall-clock 性能统计汇总、任务摘要、绘图脚本、`log_convert`、`replay`、`compare_logs`、`batch_stats`、`map_preprocess`、`batch_runner`、版本化气动表文件、固定格式线性 surrogate 只读推理、气动/地形资源/surrogate 资源 manifest 追踪、Monte Carlo 模板展开和确定性均匀/LHS 均匀/Halton 低差异均匀/正态/对数正态/截断正态/离散选择/相关正态扰动、真实闭环回放比较、覆盖率/sanitizer 构建配置、可选多实例压力 CTest 和工具测试；真实数据基准、更复杂采样策略、复杂代理模型训练/推理和真实大规模长时资源压测仍未实现 |

当前环境主循环已经使用 ECEF 真值状态、六自由度刚体状态、三轴虚拟执行机构和
RK4 积分。重力、大气、气动、推进、质量消耗和地球自转项已经通过统一环境力模型
组成 `force_b`、`moment_b` 和 ECEF 重力输入。基线启用重力、大气、低阻力气动和
地球自转，推进模型进入主链路但默认关闭。

飞控指令当前仍保留加速度级虚拟执行机构形成等效控制力，同时自动驾驶仪已经把
PNG 加速度转换为姿态命令、角速度命令和俯仰/偏航舵偏；环境气动模型会消费这些
舵偏并形成附加气动力/力矩。该链路仍是软件模拟控制律，不等同于真实弹载飞控。
四类传感器使用实例私有确定性随机流，
支持偏置、白噪声、随机游走、量化、限幅、采样保持、延迟和整帧丢包。基线导引头
延迟为 20 ms，飞控会对延迟预热、丢包或遮挡帧返回受控零指令。

基础 `faults.json` 故障脚本已经进入环境主链路，当前支持传感器偏置、
传感器线性漂移、传感器斜坡恢复偏置、传感器卡常值、传感器强制无效/丢包、
锁步安全的整帧通信窗口无效化、锁步安全的通信层帧延迟、锁步安全的上一帧重放乱序、虚拟执行机构卡滞和命令缩放，并会记录故障开始/恢复事件。
通信延迟故障使用 `target: "sensor.frame"`、`type: "COMMUNICATION_DELAY"` 或
`"FRAME_DELAY"`，`value` 为 1 到 16 的延迟步数；故障激活预热期会发送无效帧，
延迟线填满后发送滞后帧，让飞控旧帧/保持保护路径接管。
通信乱序故障使用 `target: "sensor.frame"`、`type: "COMMUNICATION_REORDER"` 或
`"FRAME_REORDER"`；激活首帧会作为预热无效帧，后续每个 LOCKSTEP 周期发送上一帧，
用于覆盖协议旧帧拒绝和控制保持路径，同时不破坏一帧一指令的闭环节拍。
通信层 `sensor.frame` 故障可配置 `recovery_hold_s`，在主故障窗口结束后继续保持
故障效果一段时间，再记录恢复事件，用于覆盖飞控超时后的恢复边界。
`summary.json` 会输出故障配置数量、触发/恢复次数、故障激活步数、传感器/执行机构
影响步数和数值诊断最大/最小值；`campaign_summary.json` 会聚合成功实例的故障和数值诊断统计。闭环回归会用同一固定种子
运行两次，并逐字节比较 `summary.json` 和 `sensor_log.bin`。环境侧会写出
`trajectory_diagnostics.csv`，记录四元数范数误差、DCM 正交性误差、质量/推进剂、
惯量、模型降级 flags、积分器和合力/力矩诊断。
飞控状态机、按 `scheduler.tasks[]` 执行的多速率调度、估计器、制导管理器、
姿态/角速度自动驾驶仪、控制分配、命令管理、变化率限制和保护状态位已经进入主链路。
飞控进程会在实例输出目录写出 `fc_internal_log.bin`，用于记录每帧保护/模式和控制中间量。
地形接口、`map.tile_path`/`map.tile_paths[]`/`map.tile_index_path` 文本/二进制空间索引瓦片加载、裸 ASCII/ESRI ASCII Grid 到内部瓦片格式的
`map_preprocess` 工具、`--index-output` 二进制索引输出和运行时固定槽位 LRU 瓦片缓存已经完成，但仓库中尚无真实 DEM 数据集，基线配置使用
`FLAT_FILL` 零高程策略。环境运行清单会记录地形启用状态、LOS 遮挡开关、缺瓦片策略、
平坦填充高度、瓦片索引路径和本实例加载的瓦片路径列表。

环境库新增 `aero_database` 气动表查询模块，支持版本化固定小端文件格式、样本 CRC、
Mach/迎角/侧滑角样本插值以及 `ERROR`、`CLAMP_AND_WARN`、`HOLD_LAST_VALID`
三种包络外策略。环境程序可通过 `aerodynamics.table_path` 加载只读气动表；
未配置时仍使用简化二次阻力/线性舵效 baseline。`run_manifest.json` 会记录气动表启用状态、
表文件路径、内部文件格式版本、包络外策略来源/覆盖值，以及可选高度/舵偏包线元数据。

`docs/design.md` 第 21 节已经把当前可执行设计收敛为工程基线，明确了环境被控对象、
传感器、飞控、多实例、GPU/并行化边界，以及 P5-P8 的完成判据。`docs/project_framework.md`
第 13 节给出了需求描述到当前实现状态的覆盖矩阵。

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
tools/log_convert/      二进制协议日志到 CSV 转换
tools/replay/           使用 sensor_log.bin 重新驱动飞控生成 command_log.bin
tools/compare_logs/     对比 sensor/command 二进制日志并输出 JSON 结果
tools/batch_stats/      聚合 summary.json 与 campaign_summary.json
tools/map_preprocess/   ASCII 高程网格到内部地形瓦片的预处理工具
tools/batch_runner/     批次清单入口，顺序调用 instance_manager 并可选聚合统计
tools/plot/             轨迹、状态时序、数值诊断和批次统计 PNG 绘图工具
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

可选覆盖率插桩构建：

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_coverage_build -DMISSILE_ENABLE_COVERAGE=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_coverage_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_coverage_build --output-on-failure
```

可选 sanitizer 构建：

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_sanitize_build -DMISSILE_ENABLE_SANITIZERS=ON
```

该选项会在配置阶段检查 ASan/UBSan 运行库是否可用；当前环境缺少 ASan 运行库时会明确失败。

可选多实例压力测试构建：

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_long_build -DMISSILE_ENABLE_LONG_TESTS=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_long_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_long_build -R long_campaign_pressure_test --output-on-failure
```

`long_campaign_pressure_test` 会启动 6 个闭环实例、并发上限 3，检查
`campaign_summary.json` 中的 completed/failed 计数、诊断汇总和 wall-clock
性能字段。它不是默认回归测试，避免拖慢常规开发循环。

当前 CTest 包含：

- `common_tests`：数学、随机数、环形缓冲区、配置和二进制协议。
- `environment_tests`：地球坐标、地形、执行机构、气动数据库、环境模型和 6DOF 积分。
- `flight_control_tests`：P6 飞控状态机、三维 PNG 方向/限幅/NaN 拒绝、自动驾驶仪内环开关、命令限幅/变化率限制、通信恢复后的安全监控恢复边界、连续帧控制品质边界和保护动作。
- `closed_loop_test`：飞控与环境双进程 UDP 闭环回归，并检查飞控内部日志、轨迹诊断、summary 诊断汇总和真实传感器日志回放比较。
- `instance_manager_test`：P7 多实例计划解析、显式随机种子、端口隔离、端口占用预检、应用层 ready 心跳、飞控早退失败路径、`STOP_ON_FAILURE` 跳过路径和任务摘要。
- `log_convert_test`：P8 `command_log.bin` 到 CSV 的协议解码转换。
- `p8_tools_test`：P8 `replay`、`compare_logs`、`batch_stats` 正常路径、诊断/气动 flags 统计聚合和截断日志失败路径。
- `map_preprocess_test`：ASCII 高程网格转内部瓦片、CRC 加载和 DEM 插值。
- `batch_runner_test`：批次清单解析、`instance_manager --runtime` 顺序调用和 Monte Carlo runtime 模板展开。
- `long_campaign_pressure_test`：可选启用，覆盖 6 实例并发压力运行和 wall-clock 汇总字段。

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
  --faults configs/baseline/faults.json \
  --runtime configs/baseline/runtime.json
```

## 多实例运行

基线 [runtime.json](configs/baseline/runtime.json) 默认定义 2 个开发测试实例，当前并发上限为 2：

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json
```

当前管理器会读取 `instances[]` 中的 `instance_id`、场景、飞控、故障配置、
显式随机种子和启用状态，并可通过 `tools.environment_program` 与
`tools.flight_control_program` 指定子程序路径。管理器为每个实例计算独立 UDP 端口并在启动前做端口唯一性和
可绑定预检。环境进程支持 `--random-seed`，`run_manifest.json` 会记录实例实际使用
的随机种子；`campaign_summary.json` 会记录每个实例的配置路径、端口、种子、退出
状态和故障统计。管理器启动环境前会等待飞控应用层 ready 心跳，并能识别
飞控在绑定前早退，并覆盖端口占用预检失败，以及 `STOP_ON_FAILURE` 下首个实例失败后跳过后续实例。

## 输出文件

默认输出位于：

```text
runs/baseline_dev_001/
  campaign_summary.json
  instance_0000/
    run_manifest.json
    event_log.txt
    fc_internal_log.bin
    sensor_log.bin
    command_log.bin
    trajectory.csv
    trajectory_diagnostics.csv
    summary.json
```

- `run_manifest.json`：软件版本、协议版本、配置路径、端口、步长、实例随机种子、气动表资源和地形资源追踪字段。
- `event_log.txt`：启动、命中、通信异常和停止事件。
- `fc_internal_log.bin`：飞控每帧模式、状态、加速度、姿态、角速度等内部记录。
- `sensor_log.bin`：固定小端线格式的完整传感器报文。
- `command_log.bin`：固定小端线格式的完整控制指令报文。
- `trajectory.csv`：导弹和目标的 ECEF/LLA 状态、AGL、速度、加速度、质量、推进剂、合力、合力矩、距离和闭合速度。
- `trajectory_diagnostics.csv`：四元数、DCM、质量/惯量、模型降级 flags、气动状态和积分器诊断。
- `summary.json`：命中结果、最小距离、最近点时刻、退出原因、故障统计和数值诊断汇总。

转换控制命令日志为 CSV：

```sh
./build/tools/log_convert/log_convert \
  --type command \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_log.csv
```

回放传感器日志重新驱动飞控：

```sh
./build/tools/replay/replay \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output runs/baseline_dev_001/instance_0000/replayed_command_log.bin
```

比较两个协议日志并输出 JSON：

```sh
./build/tools/compare_logs/compare_logs \
  --type command \
  --instance-id 0 \
  --left runs/baseline_dev_001/instance_0000/command_log.bin \
  --right runs/baseline_dev_001/instance_0000/replayed_command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_compare.json
```

聚合单实例或批次摘要：

```sh
./build/tools/batch_stats/batch_stats \
  --input runs/baseline_dev_001/campaign_summary.json \
  --output runs/baseline_dev_001/batch_stats.json
```

`batch_stats` 会聚合命中率、脱靶量、故障影响步数、诊断采样数、最大四元数范数误差、
最大 DCM 正交性误差、最小质量、最小惯量对角线、模型降级 flags 按位或、气动 flags 按位或、
气动外推采样数、campaign wall-clock 总耗时和最慢实例耗时。

把裸 ASCII 高程网格预处理为内部地形瓦片：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input terrain_grid.txt \
  --output terrain.tile \
  --width 1201 \
  --height 1201 \
  --lat-min-deg 30 \
  --lat-max-deg 31 \
  --lon-min-deg 120 \
  --lon-max-deg 121 \
  --height-scale 1.0 \
  --height-offset 0.0 \
  --index-output terrain_index.bin
```

ESRI ASCII Grid 可直接读取头部元数据：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input dem.asc \
  --input-format esri-ascii \
  --output terrain.tile \
  --nodata-fill 0 \
  --index-output terrain_index.bin
```

SRTM HGT 可从 `N30E120.hgt` 这类文件名推导 1 度瓦片边界，并按 big-endian
int16 高程样本转换；HGT void 值 `-32768` 需要显式填补：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input N30E120.hgt \
  --input-format srtm-hgt \
  --output terrain.tile \
  --nodata-fill 0 \
  --index-output terrain_index.bin
```

文本或二进制瓦片索引中的相对瓦片路径会按索引文件所在目录解析，便于把
DEM 瓦片目录整体移动到运行环境。

按清单顺序运行多个 runtime：

```sh
./build/tools/batch_runner/batch_runner \
  --manifest batch_runs.txt \
  --instance-manager ./build/tools/instance_manager/instance_manager \
  --batch-stats ./build/tools/batch_stats/batch_stats \
  --output batch_stats.json
```

从 runtime 模板生成确定性 Monte Carlo 清单：

```sh
./build/tools/batch_runner/batch_runner \
  --generate-manifest batch_runs.txt \
  --runtime-template runtime_template.json \
  --runtime-output-dir runs/mc_batch \
  --sample-count 32 \
  --base-seed 10000
```

模板中可使用 `${sample_index}`、`${random_seed}`、`${sample_output_dir}`、
`${uniform:stream:min:max}`、`${normal:stream:mean:stddev}`、
`${lhs_uniform:stream:min:max}`、`${halton_uniform:base:min:max}`、
`${lognormal:stream:mu:sigma}`、
`${truncated_normal:stream:mean:stddev:min:max}`、
`${choice:stream:option|option}` 和
`${correlated_normal:stream:base_stream:mean:stddev:rho}` 占位符；
同一 base seed、样本序号和 stream 会生成确定性均匀、LHS 均匀、Halton 低差异均匀、
正态、对数正态、截断正态、离散选择或相关正态扰动。
生成的清单第二列指向每个样本输出目录下的 `campaign_summary.json`，
可继续交给 `batch_stats` 汇总。

## 下一阶段

当前设计已经收敛，剩余开发重点是把验证基础扩展到真实数据和更高保真模型：

1. 完成真实 DEM 数据集基准、磁盘缓存和真实 DEM LOS 遮挡闭环场景。
2. 扩展带超时恢复的真实网络丢包/非锁步乱序等更完整故障类型库，以及真实数据基准、Monte Carlo 采样策略和性能测试。
3. 扩展气动表舵偏/高度维度、复杂代理模型训练/推理和真实 DEM 数据基准。

P5 基础能力已经可用；剩余工作主要是更多故障类型和真实数据资源。

## 绘图工具

推荐使用 `tools/plot/` 下的命令行绘图工具。它们读取仿真输出并生成 PNG，
适合放入报告或回归产物：

```sh
python3 tools/plot/plot_run.py \
  --instance-dir runs/baseline_dev_001/instance_0000 \
  --output-dir runs/baseline_dev_001/instance_0000/plots \
  --title-prefix baseline

python3 tools/plot/plot_campaign.py \
  --campaign runs/baseline_dev_001/campaign_summary.json \
  --output runs/baseline_dev_001/campaign_summary.png
```

`plot_run.py` 会生成 `trajectory.png`、`timeseries.png` 和 `diagnostics.png`。
绘图工具需要 Python，以及 `pandas`、`numpy`、`matplotlib`。

旧的 `scripts/plot.py`、`scripts/ploty_plot.py` 和 `scripts/earth_plot.py`
仍保留为交互式辅助脚本，但它们使用固定或手工输入路径，不适合批量报告。

静态或动画三维轨迹辅助脚本：

```sh
python3 scripts/plot.py
python3 scripts/ploty_plot.py
```

带地球球面的 ECEF 轨迹辅助脚本：

```sh
cd scripts
python3 earth_plot.py
```

## 配置文件

- [scenario.json](configs/baseline/scenario.json)：仿真步长、初始状态、地球、地图和环境模型配置。
- [flight_control.json](configs/baseline/flight_control.json)：飞控任务、比例导引和安全参数。
- [runtime.json](configs/baseline/runtime.json)：实例数量、端口、输出目录和逐实例计划。
- [faults.json](configs/baseline/faults.json)：按仿真时间触发传感器和执行机构故障。

所有配置必须是完整合法 JSON，并包含受支持的 `schema_version`。启动程序会拒绝错误 JSON、缺失当前程序要求的必填节和不支持的配置版本。

## 代码规范

- 公共接口和数据结构使用 Doxygen 风格文档注释。
- 注释说明坐标系、单位、所有权、错误条件和数值假设，不重复代码表面含义。
- 仿真主循环不进行不可控动态内存分配。
- 网络、配置和浮点输入必须校验。
- 实例之间禁止共享可变运行时状态。
- 每个阶段必须通过单元测试和双进程闭环回归后再继续扩展。
