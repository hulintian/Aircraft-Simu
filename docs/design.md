# 飞控与环境闭环仿真系统工业级设计文档

> 落地状态更新：2026-08-20。第 21 节是 P0-P8/V0-V5 的当前完成基线；
> V6 真实数据标定和 V7 HIL/实测仍属于外部证据，不计入仓库内完成度。

## 1. 项目定位

本项目使用 C 语言实现一个工业级飞控闭环仿真系统。系统由两个主程序和一个共享基础库组成：

```text
flight_control_sim/   # 飞控模拟程序，独立进程
environment_sim/      # 环境仿真程序，独立进程
common/               # 共享协议、数学库、日志、配置、诊断工具
```

项目目标不是写一个演示级二维仿真，而是构建一个具备工程扩展能力的软件在环仿真系统。系统从设计上支持：

- 三维空间弹目相对运动。
- 六自由度刚体动力学接口。
- 传感器误差、延迟、丢包和故障注入。
- 执行机构动态响应、限幅、速率限制和故障模式。
- 飞控模拟程序的任务调度、状态机、健康监测和保护逻辑。
- 二进制通信协议、接口控制文档和版本兼容。
- 可重复仿真、批量仿真、回放和离线分析。
- SIL 软件在环验证；后续可预留 HIL 半实物接口，但本项目不接真实硬件。

本项目只面向软件仿真、教学验证和工程软件架构研究。不接入真实传感器、真实执行机构或真实型号参数，不实现真实装备可直接使用的硬件驱动、总线协议或控制接口。

与 JSBSim、FlightGear、X-Plane、Microsoft Flight Simulator、MathWorks Aerospace Blockset、
NASA Trick 和 Ansys STK 等成熟平台的能力差距，见
[flight_sim_software_comparison.md](flight_sim_software_comparison.md)。该对比文档用于界定本项目
当前能宣称的能力、不能宣称的能力，以及后续工程增强优先级。

本设计的研究依据和文献边界见
[research_report_evidence_basis.md](research_report_evidence_basis.md)。该报告把公开 SIL、
仿真框架、6DOF、气动代理、轨迹优化和制导研究与本项目设计/源码逐项对应。本文吸收其中
能转化为工程约束的部分：飞控与环境进程分离、测量/指令协议边界、6DOF 力/矩主链、
数值不变量诊断、可复现实验工具链，以及气动表/代理模型的包线审计要求。

## 2. 设计原则

### 2.1 工业级工程原则

1. 核心程序使用 C11。
2. 构建系统使用 CMake。
3. 所有模块有明确边界和头文件接口。
4. 所有跨进程数据通过稳定协议交换，不共享内部结构体。
5. 所有仿真运行必须可重复，随机数由场景配置中的种子控制。
6. 主循环中禁止不可控动态内存分配。
7. 协议、配置、日志均带版本号。
8. 关键状态、故障、保护动作必须可追踪。
9. 所有模型参数来自配置文件，不硬编码在算法中。
10. 设计从一开始支持三维和六自由度接口，即使部分模型初期可使用占位实现。

### 2.2 安全边界

本项目保留仿真边界：

- 不提供真实型号气动参数。
- 不提供真实硬件驱动。
- 不实现真实作战流程。
- 不实现实装总线协议。
- 不接入真实 IMU、舵机、导引头或发动机。
- 不输出可直接接入真实设备的控制信号。

飞控程序是软件模拟件，作用是验证闭环软件结构、任务调度、传感器接口、制导计算和控制指令管理。

### 2.3 研究证据使用原则

本项目把公开文献作为**设计合理性依据**，不把文献引用误写成当前模型精度证明。
设计和文档中使用研究依据时按三档区分：

- **直接依据**：系统边界或机制与本项目同构，例如控制软件和被控对象仿真分离、
  通过 UDP/网络交换测量与控制指令的软件在环闭环。
- **方法依据**：数学形式或验证方法可迁移，但对象不同，例如 6DOF 刚体方程、
  四元数姿态推进、积分器误差和不变量诊断。
- **路线参考**：支持后续研究方向，不证明当前实现已经达到该能力，例如 AirfRANS
  类高保真气动数据集、Dymos 类最优控制工具链或在线优化制导。

因此，本文中的“工业级”指模块边界、协议、日志、配置、验证和可扩展性按工程方式组织；
不表示当前低阶气动参数、虚拟执行机构、传感器噪声或任何具体飞行对象已经通过实物标定。
凡涉及真实物理精度，必须另有解析算例、独立工具交叉验证、风洞/台架或实测数据支撑。

## 3. 系统总体架构

### 3.1 进程关系

```text
                         +----------------------+
                         |   flight_control_sim |
                         |----------------------|
                         | interface_rx         |
                         | frame_validation     |
 SensorFrame             | navigation           | ControlCommand
 +----------------------> | estimator            | +---------------------+
 |                       | guidance             |                       |
 |                       | controller           |                       |
 |                       | safety_monitor       |                       |
 |                       | interface_tx         |                       |
 |                       +----------------------+                       |
 |                                                                        |
 |                                                                        v
+----------------------+                                      +----------------------+
|   environment_sim    |                                      | binary UDP protocol  |
|----------------------|                                      +----------------------+
| scenario             |
| world_truth_state    |
| target_model         |
| missile_plant        |
| actuator_model       |
| sensor_models        |
| fault_injection      |
| recorder             |
+----------------------+
```

环境程序是仿真主时钟拥有者。每个仿真步由环境程序生成传感器帧，飞控程序基于该帧计算控制指令，环境程序再根据指令推进真实状态。

### 3.2 主项目结构

```text
missile/
  CMakeLists.txt
  docs/
    design.md
    icd.md
    verification_plan.md
  common/
    include/common/
      build_info.h
      config.h
      crc32.h
      diag.h
      logger.h
      math_constants.h
      matrix3.h
      packet.h
      protocol.h
      quaternion.h
      random.h
      ring_buffer.h
      sim_time.h
      status.h
      vec2.h
      vec3.h
    src/
    tests/
  flight_control_sim/
    include/fc/
      fc_app.h
      fc_config.h
      fc_context.h
      fc_health.h
      fc_modes.h
      fc_scheduler.h
      fc_state.h
      fc_internal_log.h
      control_quality.h
      guidance_png.h
      guidance_manager.h
      navigation.h
      estimator.h
      autopilot.h
      command_manager.h
      safety_monitor.h
    src/
    tests/
  environment_sim/
    include/env/
      env_app.h
      env_config.h
      env_context.h
      earth_model.h
      geo_coordinate.h
      map_tile.h
      terrain_model.h
      atmosphere_model.h
      gravity_model.h
      target_model.h
      missile_plant_6dof.h
      aero_model.h
      aero_database.h
      aero_database_v2.h
      aero_surrogate.h
      propulsion_model.h
      mass_model.h
      actuator_model.h
      wind_model.h
      sensor_imu.h
      sensor_accel.h
      sensor_speed.h
      sensor_seeker.h
      sensor_noise.h
      fault_injection.h
      hit_detect.h
    src/
    tests/
  tools/
    instance_manager/
    map_preprocess/
    replay/
    log_convert/
    batch_runner/
    plot/
  configs/
    baseline/
      scenario.json
      flight_control.json
      faults.json
      runtime.json
```

### 3.3 飞行实例定义

本系统定义一个“飞行实例”：

```text
FlightInstance = 1 个 environment_sim 进程 + 1 个 flight_control_sim 进程
```

每个飞行实例内部包含一套独立闭环：

```text
environment_sim(instance_id)
  -> SensorFrame(instance_id)
  -> flight_control_sim(instance_id)
  -> ControlCommand(instance_id)
  -> environment_sim(instance_id)
```

实例之间相互隔离：

- 独立仿真时间。
- 独立随机种子。
- 独立网络端口或通信通道。
- 独立配置快照。
- 独立日志目录。
- 独立命中/脱靶/故障统计。
- 不读取其他实例状态。
- 不订阅其他实例消息。
- 不依赖其他实例的启动、运行或结束结果。

同一批仿真中的所有实例由 `instance_id` 唯一标识。`instance_id` 从 `runtime.json` 分配，贯穿配置、协议、日志、事件和汇总报告。

硬性约束：实例之间不存在运行时依赖。一个实例的环境程序只与本实例的飞控程序通信；一个实例的飞控程序只接收本实例的传感器帧，只输出本实例的控制指令。任何跨实例数据读取、跨实例状态同步、跨实例控制指令复用都属于架构违规。

允许多个实例读取同一份只读静态资源，例如程序二进制、地图瓦片、默认配置模板和只读气动表。此类共享资源必须不可变，不能承载运行期状态。

### 3.4 多实例仿真架构

多实例仿真由 `tools/instance_manager` 或 `tools/batch_runner` 负责调度。核心仿真进程仍然保持简单：`environment_sim` 和 `flight_control_sim` 只处理自己的一个实例，不在进程内部管理其他实例。

推荐进程模型：

```text
instance_manager
  -> launch environment_sim --instance-id 0
  -> launch flight_control_sim --instance-id 0
  -> launch environment_sim --instance-id 1
  -> launch flight_control_sim --instance-id 1
  -> ...
  -> collect summaries
  -> write campaign_summary.json
```

这种设计避免一个环境进程内部承载多个飞行对象导致状态、日志、故障和调试互相污染。

### 3.5 多实例资源分配

每个实例需要分配：

```text
instance_id
scenario_config
flight_control_config
fault_config
random_seed
environment_port
flight_control_port
log_dir
run_mode
```

端口分配建议使用基础端口加偏移：

$$
P_{\text{env}}(i) = P_{\text{env,base}} + 2i
$$

$$
P_{\text{fc}}(i) = P_{\text{fc,base}} + 2i
$$

其中 $i$ 为 `instance_id`。

随机种子建议使用批次种子派生：

$$
s_i = \operatorname{hash}(s_{\text{campaign}}, i)
$$

这样同一批次可复现，同时不同实例不会共享随机序列。

### 3.6 多实例运行模式

多实例支持两种调度：

```text
PARALLEL
  多个 FlightInstance 并发运行，适合 Monte Carlo 和性能足够的机器。

SEQUENTIAL
  FlightInstance 串行运行，适合资源有限或需要严格复现调试的场景。
```

并发运行时，实例之间不直接通信，也不通过共享内存、共享文件或全局变量交换运行期状态。批量统计只通过实例结束后的 `summary.json` 和 `campaign_summary.json` 汇总。汇总过程不得反向影响任何实例的仿真结果。

## 4. 运行模式

系统支持以下运行模式：

```text
SIL_REALTIME
  软件在环实时模式。环境按仿真频率推进，适合联调观察。

SIL_FAST
  软件在环最快模式。环境不等待真实时间，适合批量仿真。

REPLAY_PASSIVE
  被动回放模式。只播放历史日志，不重新计算飞控。

REPLAY_WITH_FC
  使用历史传感器帧重新驱动飞控，验证飞控版本变化影响。

MONTE_CARLO
  批量随机场景模式，用于统计脱靶量、稳定性和故障覆盖。
```

所有模式都必须生成运行清单 `run_manifest`，记录配置版本、随机种子或其来源、程序版本、
协议版本和日志文件路径。`SIL_FAST`/`SIL_REALTIME` 由环境与飞控分别写实例 manifest；
`REPLAY_PASSIVE`、`REPLAY_WITH_FC` 和 `MONTE_CARLO` 由对应工具写
`*.run_manifest.json` sidecar，并记录输入/输出 CRC、帧数或样本数和工作流状态。

## 5. 时间系统

### 5.1 仿真时间

仿真时间由环境程序维护：

$$
t_k = k \Delta t
$$

其中：

- $k$ 为仿真步序号。
- $\Delta t$ 为仿真步长。

工业级设计要求所有传感器帧、控制指令、日志记录都使用仿真时间，不使用系统墙钟时间作为算法输入。

### 5.2 多速率调度

系统内部支持多速率任务。不同任务通过整数分频方式调度：

$$
f_i = \frac{f_{\text{base}}}{n_i}
$$

其中：

- $f_{\text{base}}$ 为基础调度频率。
- $n_i$ 为第 $i$ 个任务的分频系数。
- $f_i$ 为第 $i$ 个任务实际运行频率。

频率值全部来自配置文件。文档和代码中的默认值只用于仿真，不代表真实设备参数。

## 6. 环境仿真程序设计

### 6.1 环境程序职责

`environment_sim` 负责维护仿真真值状态，包含：

- 地球曲面坐标系下的导弹状态。
- 真实地图/DEM 地形数据。
- 经纬高、ECEF、局部 NED/ENU 坐标转换。
- 目标状态。
- 大气、重力、质量、推力和气动模型接口。
- 执行机构状态。
- 传感器真值和测量值。
- 故障注入状态。
- 命中、脱靶、超时和异常判定。

飞控程序不能读取这些真值，只能读取 `SensorFrame`。

### 6.2 坐标系

系统从设计上支持以下坐标系：

```text
LLA 系：大地坐标，经度、纬度、高程
ECEF 系：地心地固坐标系
NED 系：局部北东地坐标系
ENU 系：局部东北天坐标系
B 系：弹体系
V 系：速度坐标系
L 系：弹目视线坐标系
```

环境真值主状态使用 `ECEF` 坐标。配置、地图和日志显示使用 `LLA`。短距离控制、视景和局部分析可派生 `NED` 或 `ENU` 坐标，但不能把局部平面坐标作为全局真值。

姿态使用四元数作为主表示，欧拉角只用于日志和显示。

四元数归一化约束：

$$
\lVert \mathbf q \rVert = 1
$$

方向余弦矩阵由四元数计算：

$$
\mathbf C_{BI} = \operatorname{DCM}(\mathbf q_{BI})
$$

### 6.3 地球模型

环境模型按真实地图的球形地球思路设计。工业级实现建议以 WGS-84 椭球作为默认地球模型；如果确实需要球形地球，可通过配置切换到等效球体模型。

WGS-84 参数：

$$
a = 6378137.0\ \text{m}
$$

$$
f = \frac{1}{298.257223563}
$$

第一偏心率平方：

$$
e^2 = f(2-f)
$$

卯酉圈曲率半径：

$$
N(\varphi)
=
\frac{a}
{\sqrt{1-e^2\sin^2\varphi}}
$$

其中 $\varphi$ 为大地纬度。

### 6.4 LLA 与 ECEF 转换

大地坐标：

$$
\mathbf l =
\begin{bmatrix}
\varphi \\
\lambda \\
h
\end{bmatrix}
$$

其中：

- $\varphi$：纬度。
- $\lambda$：经度。
- $h$：相对参考椭球的高程。

LLA 转 ECEF：

$$
x =
\left(N(\varphi)+h\right)
\cos\varphi
\cos\lambda
$$

$$
y =
\left(N(\varphi)+h\right)
\cos\varphi
\sin\lambda
$$

$$
z =
\left(N(\varphi)(1-e^2)+h\right)
\sin\varphi
$$

ECEF 到 LLA 使用迭代解算或 Bowring 类闭式近似，封装在 `geo_coordinate.c` 中，不允许业务模块自行实现转换。

### 6.5 局部 NED/ENU 坐标

每个场景配置一个局部参考点：

$$
\mathbf l_0 =
\begin{bmatrix}
\varphi_0 \\
\lambda_0 \\
h_0
\end{bmatrix}
$$

对应 ECEF 原点：

$$
\mathbf r_0 = \operatorname{LLA2ECEF}(\mathbf l_0)
$$

任一点 ECEF 位置 $\mathbf r_e$ 到局部坐标的相对向量：

$$
\Delta \mathbf r_e = \mathbf r_e - \mathbf r_0
$$

ECEF 到 ENU 的旋转矩阵：

$$
\mathbf R_{ENU,ECEF}
=
\begin{bmatrix}
-\sin\lambda_0 & \cos\lambda_0 & 0 \\
-\sin\varphi_0\cos\lambda_0 & -\sin\varphi_0\sin\lambda_0 & \cos\varphi_0 \\
\cos\varphi_0\cos\lambda_0 & \cos\varphi_0\sin\lambda_0 & \sin\varphi_0
\end{bmatrix}
$$

局部 ENU 坐标：

$$
\mathbf r_{enu}
=
\mathbf R_{ENU,ECEF}
\Delta \mathbf r_e
$$

NED 与 ENU 的关系：

$$
\mathbf r_{ned}
=
\begin{bmatrix}
r_{enu,y} \\
r_{enu,x} \\
-r_{enu,z}
\end{bmatrix}
$$

### 6.6 真实地图与地形模型

环境程序需要把地表建成曲面地图，不使用无限平面地面。地图系统由 `map_tile` 和 `terrain_model` 管理。

地图数据层级：

```text
MapDatabase
  -> TileIndex
  -> TerrainTile
  -> ElevationGrid
  -> Material/SurfaceMask
```

核心能力：

```text
1. 从 `map.tile_path`、`map.tile_paths[]` 或 `map.tile_index_path` 加载地形瓦片。
2. 查询指定经纬度的地形高程。
3. 插值 DEM 网格。
4. 计算离地高度 AGL。
5. 进行地表碰撞判定。
6. 为导引头/视景预留地形遮挡查询。
```

地形高度函数：

$$
h_{\text{terrain}}
=
H(\varphi,\lambda)
$$

飞行器相对地高度：

$$
h_{\text{AGL}}
=
h - h_{\text{terrain}}
$$

地表碰撞判据：

$$
h_{\text{AGL}} \le 0
$$

DEM 网格双线性插值：

$$
H(u,v)
=
(1-u)(1-v)H_{00}
+
u(1-v)H_{10}
+
(1-u)vH_{01}
+
uvH_{11}
$$

其中 $u,v\in[0,1]$ 为瓦片内归一化坐标。

### 6.7 地图数据格式

核心仿真程序不直接解析复杂地图源文件。工业级流程采用离线预处理：

```text
原始地图/DEM 数据
  -> tools/map_preprocess
  -> 内部二进制瓦片格式
  -> environment_sim 运行时按需加载
```

当前 `tools/map_preprocess` 支持三类输入：

```text
raw-grid
  裸高程矩阵，宽高和经纬度范围由命令行给出。

esri-ascii
  ESRI ASCII Grid，经纬度范围由 ncols/nrows/xllcorner/yllcorner/cellsize 推导。
  样本网格会从 ESRI 的北到南行序转换为内部的 lat_min 到 lat_max 行序。
  NODATA 样本默认拒绝；显式配置 `--nodata-fill` 时才使用指定高度填补。

srtm-hgt
  SRTM HGT big-endian int16 高程文件，网格尺寸由文件大小推导，经纬度范围由
  `N30E120.hgt` 这类文件名推导为 1 度瓦片。样本网格会从 HGT 的北到南行序转换为
  内部的 lat_min 到 lat_max 行序；void 值 `-32768` 需要显式 `--nodata-fill`。
```

工具可通过 `--index-output tile_index.bin` 同时写出二进制空间索引，索引项包含瓦片
经纬度边界和内部瓦片路径，用于运行时加载前筛选候选瓦片。

内部瓦片建议包含：

```c
typedef struct TerrainTileHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t grid_width;
    uint16_t grid_height;
    double lat_min;
    double lat_max;
    double lon_min;
    double lon_max;
    double height_scale;
    double height_offset;
    uint32_t data_crc32;
} TerrainTileHeader;
```

运行时地形查询接口：

```c
SimStatus terrain_get_height(
    const TerrainModel *terrain,
    double lat_rad,
    double lon_rad,
    double *height_m
);
```

### 6.8 地图预处理要求

`tools/map_preprocess` 负责把外部地图/DEM 数据转换为内部瓦片。核心仿真程序不直接依赖外部 GIS 数据格式。

预处理输出：

```text
tile_index.txt
  -> 文本瓦片索引，每行一个内部瓦片路径，支持空行和 # 注释

tile_index.bin
  -> 固定小端二进制空间索引，包含 magic、version、entry_count、
     每个瓦片的 lat/lon 边界和路径

tile_*.bin
  -> 高程网格、地表分类、有效性掩码

map_manifest.json
  -> 原始数据来源、生成时间、坐标基准、分辨率、工具版本
```

当前运行时支持 `map.tile_index_path` 指向上述文本索引或二进制空间索引，并把索引文件路径和展开后的
瓦片路径列表写入 `run_manifest.json`。`map.terrain.cache_tile_count` 控制单实例固定槽位
LRU 缓存容量；查询路径会按覆盖关系懒加载内部瓦片，二进制索引在加载前用边界筛选候选项，
索引内相对瓦片路径按索引文件所在目录解析，容量满时淘汰最久未使用槽位。
`summary.json` 记录缓存路径数、容量、当前加载数、累计加载数和淘汰数。
仓库提供珠峰附近 9x9 真实来源 DEM fixture 和 `map_manifest.json`，记录来源 URL、
访问日期、坐标范围与 SHA-256。预处理测试覆盖峰值和阻挡/放通 LOS，双进程闭环会
加载其派生瓦片验证遮挡保护。生产地图仍需单独维护许可证、完整覆盖和资源清单。

瓦片索引项建议：

```c
typedef struct TerrainTileIndexEntry {
    double lat_min;
    double lat_max;
    double lon_min;
    double lon_max;
    uint32_t grid_width;
    uint32_t grid_height;
    uint64_t file_offset;
    uint32_t file_size;
    uint32_t crc32;
} TerrainTileIndexEntry;
```

缺失瓦片策略：

```text
MAP_MISSING_ERROR
  缺瓦片立即终止仿真。

MAP_MISSING_FLAT_FILL
  使用配置高度填充，并记录严重告警。

MAP_MISSING_NEAREST
  使用最近有效瓦片边界值外推，并记录告警。
```

工业级回归测试建议使用 `MAP_MISSING_ERROR`，避免无意中在错误地形上完成仿真。

### 6.9 球形地图下的视线和遮挡

弹目相对向量必须在 ECEF 中计算：

$$
\mathbf r_{mt,e}
=
\mathbf p_{t,e}
-
\mathbf p_{m,e}
$$

弹目距离：

$$
R
=
\left\|
\mathbf r_{mt,e}
\right\|
$$

如果需要判断地形遮挡，沿弹目连线采样：

$$
\mathbf p_e(s)
=
\mathbf p_{m,e}
+
s
\left(
\mathbf p_{t,e}
-
\mathbf p_{m,e}
\right),
\quad
s\in[0,1]
$$

将 $\mathbf p_e(s)$ 转为 LLA，查询地形高度 $H(\varphi,\lambda)$。若某采样点满足：

$$
h(s) - H(\varphi(s),\lambda(s)) \le 0
$$

则视线被地形遮挡。

### 6.10 六自由度真值状态

环境程序的导弹真值状态按六自由度刚体接口设计：

```c
typedef struct PlantState6Dof {
    double time;

    Vec3 pos_ecef;
    Vec3 vel_ecef;
    Vec3 accel_ecef;

    double lat_rad;
    double lon_rad;
    double height_m;
    double height_agl_m;

    Quat q_bi;
    Vec3 omega_b;
    Vec3 alpha_b;

    double mass;
    Matrix3 inertia_b;

    Vec3 force_b;
    Vec3 moment_b;

    double actuator_pos[SIM_MAX_ACTUATORS];
    double actuator_rate[SIM_MAX_ACTUATORS];
} PlantState6Dof;
```

### 6.11 六自由度运动方程接口

平动方程：

$$
\dot{\mathbf r}_e = \mathbf v_e
$$

$$
\dot{\mathbf v}_e =
\frac{1}{m}\mathbf C_{eB}\mathbf F_B
+ \mathbf g_e
+ \mathbf a_{\text{earth}}
$$

转动方程：

$$
\mathbf I_B \dot{\boldsymbol\omega}_B
+
\boldsymbol\omega_B \times
\left(\mathbf I_B \boldsymbol\omega_B\right)
=
\mathbf M_B
$$

四元数运动学：

$$
\dot{\mathbf q}_{BI}
=
\frac{1}{2}
\mathbf \Omega(\boldsymbol\omega_B)
\mathbf q_{BI}
$$

其中：

- $\mathbf r_e$、$\mathbf v_e$ 为 ECEF 位置和速度。
- $\mathbf F_B$、$\mathbf M_B$ 分别为弹体系合力和合力矩。
- $\mathbf g_e$ 为 ECEF 下重力加速度。
- $\mathbf a_{\text{earth}}$ 为地球自转相关修正项，可通过配置启用或关闭。

气动、推力、重力、执行机构等模型只通过该接口汇总，不在积分器内部硬编码。

质量和惯量必须作为动力学状态的一部分被校验，而不是作为静态常量假设：

```text
mass > 0
propellant_mass >= 0
inertia_b symmetric positive definite
quat_norm(q_bi) ~= 1
```

当前实现保留按总质量比例缩放惯量的兼容路径，并提供可选精细质量属性模型：
干体质心/惯量、满装与空箱推进剂质心、推进剂惯量按剩余比例演化，再用平行轴定理
合成总质心和完整 3x3 惯量张量。质量、质心、惯量和质量流量由同一模型输出并校验
对称正定性，避免推进、气动和积分器维护不一致状态。

### 6.12 模型分层

环境模型按以下顺序计算：

```text
earth_model
  -> 更新地球参数、坐标转换上下文

map_tile_manager
  -> 按当前位置加载真实地图/DEM 瓦片

target_model
  -> 更新目标真值状态

guidance_command_input
  -> 读取飞控指令

actuator_model
  -> 将指令转换为虚拟舵面/执行机构状态

propulsion_model
  -> 输出推力项，可配置关闭

aero_model
  -> 根据状态和执行机构输出气动力/力矩

gravity_model
  -> 输出重力加速度

missile_plant_integrator
  -> 积分六自由度真值状态

terrain_model
  -> 更新离地高度、地表碰撞和遮挡信息

sensor_models
  -> 从真值状态生成测量帧
```

当前 `target_model` 支持两种可执行模型：

- `CONSTANT_VELOCITY`：ECEF 匀速真值。
- `SCRIPTED_ACCELERATION`：按 `target.maneuvers[]` 配置 ECEF 常加速度段；若机动开始或
  结束落在一个固定步内，模型会在边界处分段精确积分。重叠机动段按加速度向量求和。

`SCRIPTED_ACCELERATION` 必须至少包含一个合法机动段；不能再用空 `SCRIPTED` 配置冒充
机动目标。命中与最近点使用一个仿真步内的相对线段连续判定，而不是只比较步端距离，
因此高速穿越不会因两个端点均在命中半径外而漏判。完整目标飞行动力学和行为库仍属于
V6 场景模型扩展。

### 6.13 气动模型分层与包线管理

气动模型按保真度分层，所有层级对积分器输出同一接口：

```c
typedef struct AeroInput {
    double mach;
    double alpha_rad;
    double beta_rad;
    double height_m;
    double reynolds;
    double actuator_rad[SIM_MAX_ACTUATORS];
    Vec3 velocity_b_mps;
} AeroInput;

typedef struct AeroOutput {
    Vec3 force_b_n;
    Vec3 moment_b_nm;
    uint32_t model_flags;
    double uncertainty_scale;
} AeroOutput;
```

气动模型层级：

```text
AERO_SIMPLE
  当前低阶阻力/控制力/控制力矩模型。用于闭环调试、单元测试和无数据基线。

AERO_TABLE
  Mach / AoA / beta / 舵偏 / 高度或 Reynolds 数表格插值模型。
  这是 P8 之后优先接入的工程主模型。

AERO_SURROGATE
  当前已接入固定文本格式的线性只读代理推理；后续复杂代理模型仍必须离线训练。
  在线只允许只读推理，不允许在主循环训练或动态修改权重。
```

所有非 `AERO_SIMPLE` 模型必须声明适用包线：

```text
mach_min / mach_max
alpha_min_rad / alpha_max_rad
beta_min_rad / beta_max_rad
height_min_m / height_max_m
actuator_min_rad / actuator_max_rad
```

当输入超出包线时，模型不能静默外推。必须按配置执行以下策略之一：

```text
ERROR
  拒绝本步仿真并记录错误。

CLAMP_AND_WARN
  钳位到包线边界，置 `AERO_FLAG_EXTRAPOLATED` 并写事件日志。

HOLD_LAST_VALID
  使用上一帧有效气动输出，置降级标志。
```

气动数据库文件应是只读资源，可由多个实例共享，但运行期不得写入。表格插值和代理
模型推理必须是确定性的；同一配置、同一随机种子和同一输入轨迹应产生逐帧一致的
气动力和力矩。

### 6.14 数值积分器

积分器需要统一接口：

```c
typedef enum IntegratorType {
    INTEGRATOR_EULER = 1,
    INTEGRATOR_RK2,
    INTEGRATOR_RK4
} IntegratorType;
```

工业级默认应支持 RK4。欧拉积分仅用于调试和对照。

状态推进抽象为：

$$
\mathbf x_{k+1}
=
\Phi
\left(
\mathbf x_k,
\mathbf u_k,
\Delta t
\right)
$$

其中 $\Phi$ 由选择的积分器和动力学模型共同决定。

积分器必须提供诊断输出，至少包括：

```text
quat_norm_error
dcm_orthogonality_error
mass_positive
inertia_positive_definite
max_force_norm
max_moment_norm
aero_model_flags
model_degradation_flags
integrator_type
dt_s
```

这些诊断不参与控制闭环，但必须进入轨迹或诊断日志，用于回归、批量统计和数值健康
审计。固定场景下应支持 Euler/RK2/RK4 对照测试；RK4 是默认工业基线，Euler 仅用于
调试和误差趋势对照。

## 7. 执行机构模型

### 7.1 执行机构状态

执行机构模型不直接把飞控指令当作真实舵偏，而是经过动态环节：

```c
typedef struct ActuatorState {
    double cmd;
    double pos;
    double rate;
    double pos_min;
    double pos_max;
    double rate_limit;
    double time_constant;
    uint32_t fault_flags;
} ActuatorState;
```

### 7.2 一阶动态与限幅

一阶执行机构模型：

$$
\dot{\delta}
=
\frac{\delta_c - \delta}{\tau_a}
$$

离散形式：

$$
\delta_{k+1}^{*}
=
\delta_k
+
\frac{\Delta t}{\tau_a}
\left(
\delta_{c,k} - \delta_k
\right)
$$

位置限幅：

$$
\delta_{k+1}
=
\operatorname{sat}
\left(
\delta_{k+1}^{*},
\delta_{\min},
\delta_{\max}
\right)
$$

速率限制：

$$
\left|
\frac{\delta_{k+1}-\delta_k}{\Delta t}
\right|
\le
\dot{\delta}_{\max}
$$

### 7.3 故障模式

执行机构支持故障注入：

```text
ACTUATOR_FAULT_STUCK
ACTUATOR_FAULT_BIAS
ACTUATOR_FAULT_RATE_LIMIT_DEGRADED
ACTUATOR_FAULT_POSITION_LIMIT_DEGRADED
ACTUATOR_FAULT_DELAY
```

故障注入由环境程序配置，不由飞控程序直接控制。

## 8. 传感器模型

### 8.1 传感器清单

工业级仿真中，环境程序至少模拟以下传感器或测量通道：

```text
IMU 陀螺仪：
  角速度测量

IMU 加速度计：
  比力或加速度测量

速度测量：
  速度大小或速度向量测量

导引头/目标测量器：
  弹目距离
  视线方向
  视线角速度
  闭合速度

状态帧诊断：
  测量有效性
  饱和标志
  丢帧标志
  时间戳
```

比例导引必须依赖目标相对测量。仅有陀螺仪、速度表和加速度表不足以完成比例导引。

### 8.2 传感器误差模型

统一测量模型：

$$
\mathbf z_k
=
\mathbf h(\mathbf x_k)
+
\mathbf b_k
+
\boldsymbol\eta_k
+
\mathbf w_k
$$

其中：

- $\mathbf z_k$ 为传感器测量。
- $\mathbf h(\mathbf x_k)$ 为真值到测量的观测函数。
- $\mathbf b_k$ 为零偏。
- $\boldsymbol\eta_k$ 为白噪声。
- $\mathbf w_k$ 为随机游走误差。

随机游走：

$$
\mathbf w_{k+1}
=
\mathbf w_k
+
\boldsymbol\nu_k \sqrt{\Delta t}
$$

### 8.3 延迟与采样

传感器输出必须支持不同采样周期：

$$
T_s^{(i)} = n_i \Delta t
$$

延迟通过环形缓冲区实现：

$$
z_{\text{out}}(t_k)
=
z_{\text{raw}}(t_k - \tau_d)
$$

### 8.4 导引头相对测量

设导弹位置、速度为 $\mathbf p_m$、$\mathbf v_m$，目标位置、速度为 $\mathbf p_t$、$\mathbf v_t$。

相对位置：

$$
\mathbf r = \mathbf p_t - \mathbf p_m
$$

相对速度：

$$
\mathbf v_r = \mathbf v_t - \mathbf v_m
$$

弹目距离：

$$
R = \lVert \mathbf r \rVert
$$

视线单位向量：

$$
\hat{\mathbf r}
=
\frac{\mathbf r}{R}
$$

闭合速度：

$$
V_c
=
-\hat{\mathbf r}\cdot\mathbf v_r
$$

视线角速度向量：

$$
\boldsymbol\omega_{\text{LOS}}
=
\frac{\mathbf r \times \mathbf v_r}{R^2}
$$

该测量经过噪声、延迟、量化和有效性检查后进入 `SensorFrame`。

## 9. 飞控模拟程序设计

### 9.1 飞控职责

`flight_control_sim` 是软件飞控模拟件，负责：

- 接收传感器帧。
- 检查协议、时间戳、序号和状态位。
- 维护飞控内部状态。
- 执行导航和估计。
- 执行制导律。
- 执行控制律或控制指令管理。
- 执行故障检测和保护。
- 输出控制指令。
- 记录飞控内部日志。

飞控不访问环境真值。

### 9.2 飞控状态机

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

状态切换必须集中在 `fc_modes.c` 中实现，不允许在各业务模块中分散切换。

### 9.3 飞控任务架构

飞控任务采用表驱动调度：

```c
typedef struct FcTask {
    const char *name;
    uint32_t period_ticks;
    uint32_t last_run_tick;
    int (*run)(FcContext *ctx);
} FcTask;
```

典型任务：

```text
fc_task_receive
fc_task_validate
fc_task_navigation
fc_task_estimator
fc_task_guidance
fc_task_controller
fc_task_safety
fc_task_transmit
fc_task_log
```

### 9.4 导航与估计

导航层从传感器帧中提取可用测量，并维护统一导航状态：

```c
typedef struct NavState {
    double time;
    Vec3 missile_pos_ecef_est;
    Vec3 missile_vel_ecef_est;
    Vec3 missile_accel_ecef_est;
    Vec3 omega_b_est;

    double range;
    Vec3 los_unit_ecef;
    Vec3 los_rate_ecef;
    double closing_velocity;

    double lat_rad;
    double lon_rad;
    double height_m;
    double height_agl_m;

    uint32_t valid_flags;
} NavState;
```

估计器第一阶段可以使用测量直通，但接口按滤波器设计：

$$
\hat{\mathbf x}_{k+1}
=
f
\left(
\hat{\mathbf x}_k,
\mathbf z_k,
\mathbf u_k,
\Delta t
\right)
$$

后续可替换为互补滤波、卡尔曼滤波或自定义状态估计器。

### 9.5 三维比例导引

飞控制导层默认采用三维比例导引接口。

输入：

```c
typedef struct GuidanceInput {
    double range;
    double closing_velocity;
    Vec3 los_unit_ecef;
    Vec3 los_rate_ecef;
    Vec3 missile_vel_ecef;
    uint32_t valid_flags;
} GuidanceInput;
```

三维比例导引加速度指令：

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

其中：

- $N$ 为导航比。
- $V_c$ 为闭合速度。
- $\boldsymbol\omega_{\text{LOS}}$ 为视线角速度向量。
- $\hat{\mathbf r}$ 为视线单位向量。

指令限幅：

$$
\mathbf a_c^{\text{lim}}
=
\operatorname{limit\_norm}
\left(
\mathbf a_c,
a_{\max}
\right)
$$

变化率限制：

$$
\left\|
\frac{
\mathbf a_{c,k}^{\text{lim}}
-
\mathbf a_{c,k-1}^{\text{lim}}
}{\Delta t}
\right\|
\le
\dot a_{\max}
$$

### 9.6 控制指令管理

飞控输出不直接等价于真实舵偏。控制指令按层级定义：

```text
GuidanceCommand
  -> 三维期望加速度

AutopilotCommand
  -> 期望姿态/期望角速度/虚拟控制量

ControlCommand
  -> 发送给环境程序的标准控制帧
```

初期环境程序可以接收加速度级指令，后续可切换为执行机构级指令。协议字段必须提前预留。

### 9.7 飞控保护

飞控保护动作：

```text
输入 NaN/Inf：拒绝本帧
旧帧：拒绝本帧
传感器超时：进入 COMMAND_HOLD
目标测量无效：保持上一帧或退出制导
闭合速度异常：限制制导输出
指令超限：限幅并置状态位
连续异常：进入 FC_FAULT
```

保护输出状态必须写入 `command_status`。

## 10. 通信协议与 ICD

### 10.1 协议要求

跨进程通信使用二进制协议。所有帧必须包含：

- magic。
- 协议版本。
- 消息类型。
- 实例 ID。
- 序号。
- 仿真时间。
- payload 长度。
- payload 校验。

### 10.2 包头

```c
#define SIM_PACKET_MAGIC 0x53494D31u

typedef struct PacketHeader {
    uint32_t magic;
    uint16_t version_major;
    uint16_t version_minor;
    uint16_t type;
    uint16_t header_size;
    uint32_t instance_id;
    uint32_t seq;
    double sim_time;
    uint32_t payload_size;
    uint32_t payload_crc32;
} PacketHeader;
```

### 10.3 消息类型

```c
typedef enum PacketType {
    PACKET_SENSOR_FRAME     = 1,
    PACKET_CONTROL_COMMAND  = 2,
    PACKET_HEARTBEAT        = 3,
    PACKET_SIM_CONTROL      = 4,
    PACKET_EVENT            = 5
} PacketType;
```

### 10.4 SensorFrame

`SensorFrame` 是环境到飞控的唯一数据入口。

`instance_id` 放在 `PacketHeader` 中，payload 内不重复存储。飞控程序必须检查包头 `instance_id` 是否等于自身实例 ID，不匹配则丢弃。

```c
typedef struct SensorFrame {
    uint32_t seq;
    double sim_time;
    double dt;

    Vec3 missile_vel_ecef_meas;
    Vec3 missile_accel_ecef_meas;
    Vec3 missile_gyro_b_meas;

    double missile_lat_rad_meas;
    double missile_lon_rad_meas;
    double missile_height_m_meas;
    double missile_height_agl_m_meas;

    double target_range_meas;
    Vec3 target_los_unit_ecef_meas;
    Vec3 target_los_rate_ecef_meas;
    double target_closing_velocity_meas;

    uint32_t sensor_valid_flags;
    uint32_t sensor_fault_flags;
} SensorFrame;
```

### 10.5 ControlCommand

`ControlCommand` 是飞控到环境的唯一控制入口。

`ControlCommand` 同样通过 `PacketHeader.instance_id` 绑定到对应环境实例。环境程序必须拒绝其他实例的控制包。

```c
typedef struct ControlCommand {
    uint32_t seq;
    double sim_time;

    Vec3 accel_cmd_ecef;
    Vec3 attitude_cmd;
    Vec3 body_rate_cmd;

    double actuator_cmd[SIM_MAX_ACTUATORS];

    uint32_t command_mode;
    uint32_t command_status;
} ControlCommand;
```

环境程序通过配置决定使用哪一级指令：

```text
COMMAND_LEVEL_ACCELERATION
COMMAND_LEVEL_ATTITUDE
COMMAND_LEVEL_BODY_RATE
COMMAND_LEVEL_ACTUATOR
```

### 10.6 协议兼容策略

协议使用主版本和次版本：

```text
major 不一致：拒绝通信
minor 不一致：允许兼容字段缺省，但必须记录警告
```

所有新增字段只能追加，不能改变已有字段含义。

## 11. 主循环设计

### 11.1 环境程序主循环

```text
env_load_config
env_init_models
env_wait_fc_ready

while env_not_finished:
    env_update_target_truth
    env_read_last_command
    env_update_actuators
    env_compute_forces_and_moments
    env_integrate_plant
    env_update_hit_miss_status
    env_generate_sensor_frame
    env_send_sensor_frame
    env_record_trajectory
    env_record_sensor_log
    env_advance_time

env_write_summary
env_shutdown
```

### 11.2 飞控程序主循环

```text
fc_load_config
fc_init_context
fc_wait_sensor

while fc_running:
    fc_receive_sensor_frame
    fc_validate_frame
    fc_run_scheduler
    fc_send_control_command
    fc_record_internal_log

fc_shutdown
```

### 11.3 同步策略

环境程序可配置为：

```text
LOCKSTEP
  每发送一帧传感器，等待对应控制指令。

FREE_RUNNING
  环境按固定步长运行，飞控迟到时使用上一帧指令。
```

工业级回归测试建议使用 `LOCKSTEP`，实时演示可使用 `FREE_RUNNING`。

### 11.4 实例管理器主循环

`instance_manager` 不参与单个实例的物理仿真和飞控计算，只负责多实例生命周期管理。

```text
manager_load_runtime_config
manager_build_instance_plan
manager_allocate_ports
manager_create_log_dirs

while instances_remaining:
    manager_launch_ready_instances
    manager_poll_process_status
    manager_collect_finished_summaries
    manager_restart_or_mark_failed
    manager_respect_max_parallel_instances

manager_write_campaign_summary
manager_shutdown
```

实例状态机：

```text
INSTANCE_PENDING
INSTANCE_LAUNCHING
INSTANCE_RUNNING
INSTANCE_COMPLETED
INSTANCE_FAILED
INSTANCE_TIMEOUT
INSTANCE_ABORTED
```

失败处理策略：

```text
CONTINUE_ON_FAILURE
  记录失败实例，继续运行其他实例。默认策略。

RETRY_ONCE
  实例失败后只重试该实例一次，仍失败则标记失败。不得重启或影响其他实例。

ABORT_BY_OPERATOR
  由用户或外部调度器主动中止整个批次，不由某个实例的仿真状态自动触发。
```

## 12. 配置管理

### 12.1 配置格式

配置文件统一使用 JSON。C 侧通过 `common/config` 封装 JSON 解析库，业务模块不直接依赖具体第三方库 API。这样后续可以在 `cJSON`、`jsmn` 或其他 C JSON 库之间切换，而不影响飞控和环境模块。

配置读取流程：

```text
读取 JSON 文件
  -> 解析为 ConfigTree
  -> 校验 schema_version
  -> 填充默认值
  -> 做范围检查
  -> 生成 ConfigSnapshot
  -> 计算 config_crc32
```

运行主循环只能读取 `ConfigSnapshot`，不能在仿真过程中重新解析 JSON。

### 12.2 配置文件组织

为避免配置文件过多，配置简化为四个 JSON 文件：

| 文件 | 读取方 | 内容 |
|---|---|---|
| `scenario.json` | `environment_sim` | 环境、地球、地图、目标、弹体和传感器 |
| `flight_control.json` | `flight_control_sim` | 飞控任务调度、制导、控制和保护 |
| `faults.json` | `environment_sim` | 故障注入脚本 |
| `runtime.json` | 两个进程 | 网络、日志、运行模式和工具选项 |

其中 `scenario.json` 是环境程序主配置，`flight_control.json` 是飞控程序主配置。`runtime.json` 由两个进程共同读取，保证网络端口、协议版本和日志目录一致。

多实例运行时，随机种子以 `runtime.json` 中的实例配置为准；`scenario.json` 不直接配置随机种子，避免同一场景在多个实例中误用相同随机序列。

### 12.3 scenario.json 示例

```json
{
  "schema_version": 1,
  "simulation": {
    "mode": "LOCKSTEP",
    "dt": 0.01,
    "max_time": 120.0
  },
  "earth": {
    "model": "WGS84",
    "enable_rotation_terms": true,
    "origin": {
      "lat_deg": 30.0,
      "lon_deg": 120.0,
      "height_m": 0.0,
      "local_frame": "NED"
    }
  },
  "map": {
    "database_path": "data/maps/internal_tiles",
    "tile_index_path": "data/maps/tile_index.bin",
    "enable_terrain": true,
    "enable_los_occlusion": true,
    "missing_tile_policy": "ERROR",
    "terrain": {
      "interpolation": "BILINEAR",
      "height_reference": "ELLIPSOID",
      "cache_tile_count": 16
    }
  },
  "plant": {
    "model": "SIX_DOF",
    "integrator": "RK4",
    "mass_kg": 100.0,
    "propellant_mass_kg": 0.0,
    "inertia_diag": [1.0, 1.0, 1.0]
  },
  "gravity": {
    "enabled": true
  },
  "atmosphere": {
    "enabled": true,
    "maximum_model_height_m": 11000.0,
    "wind_velocity_ecef_mps": [0.0, 0.0, 0.0]
  },
  "propulsion": {
    "enabled": false,
    "thrust_n": 0.0,
    "mass_flow_kgps": 0.0,
    "burn_time_s": 0.0,
    "thrust_direction_b": [1.0, 0.0, 0.0]
  },
  "aerodynamics": {
    "enabled": true,
    "model": "SIMPLE",
    "reference_area_m2": 0.01,
    "reference_length_m": 1.0,
    "drag_coefficient": 0.1,
    "control_force_coefficient": 0.0,
    "control_moment_coefficient": 0.0,
    "database": {
      "path": "data/aero/example_table.csv",
      "inputs": ["mach", "alpha_rad", "beta_rad", "pitch_fin_rad", "yaw_fin_rad"],
      "outputs": ["cx", "cy", "cz", "cl", "cm", "cn"],
      "extrapolation_policy": "CLAMP_AND_WARN"
    },
    "envelope": {
      "mach": [0.0, 5.0],
      "alpha_rad": [-0.35, 0.35],
      "beta_rad": [-0.35, 0.35],
      "height_m": [0.0, 30000.0]
    }
  },
  "target": {
    "model": "SCRIPTED_ACCELERATION",
    "initial_lla_deg_m": [30.02, 120.05, 1000.0],
    "initial_velocity_ecef_mps": [0.0, 0.0, 0.0],
    "maneuvers": [
      {
        "start_time_s": 2.0,
        "duration_s": 1.5,
        "acceleration_ecef_mps2": [0.0, 15.0, 0.0]
      }
    ]
  },
  "sensors": {
    "imu": {
      "enabled": true,
      "sample_period_s": 0.01,
      "delay_s": 0.0,
      "dropout_probability": 0.0,
      "noise": {
        "bias_xyz": [0.0, 0.0, 0.0],
        "white_noise_std": 0.0001,
        "random_walk_std": 0.000001,
        "resolution": 0.000001
      }
    },
    "accelerometer": {
      "enabled": true,
      "sample_period_s": 0.01,
      "delay_s": 0.0,
      "dropout_probability": 0.0,
      "noise": {
        "bias_xyz": [0.0, 0.0, 0.0],
        "white_noise_std": 0.01,
        "random_walk_std": 0.0001,
        "resolution": 0.001
      }
    },
    "speedometer": {
      "enabled": true,
      "sample_period_s": 0.01,
      "delay_s": 0.0,
      "dropout_probability": 0.0,
      "noise": {
        "bias_xyz": [0.0, 0.0, 0.0],
        "white_noise_std": 0.05,
        "random_walk_std": 0.001,
        "resolution": 0.01
      }
    },
    "seeker": {
      "enabled": true,
      "sample_period_s": 0.01,
      "delay_s": 0.02,
      "dropout_probability": 0.0,
      "range_noise": {
        "bias": 0.0,
        "white_noise_std": 1.0,
        "random_walk_std": 0.01,
        "resolution": 0.01
      },
      "los_unit_noise": {
        "bias_xyz": [0.0, 0.0, 0.0],
        "white_noise_std": 0.000001,
        "random_walk_std": 0.00000001
      },
      "los_rate_noise": {
        "bias_xyz": [0.0, 0.0, 0.0],
        "white_noise_std": 0.00005,
        "random_walk_std": 0.0000001,
        "resolution": 0.0000001
      },
      "closing_velocity_noise": {
        "bias": 0.0,
        "white_noise_std": 0.1,
        "random_walk_std": 0.001,
        "resolution": 0.01
      }
    }
  }
}
```

### 12.4 flight_control.json 示例

```json
{
  "schema_version": 1,
  "scheduler": {
    "base_rate_hz": 100,
    "tasks": [
      { "name": "receive", "period_ticks": 1 },
      { "name": "navigation", "period_ticks": 1 },
      { "name": "guidance", "period_ticks": 1 },
      { "name": "controller", "period_ticks": 1 },
      { "name": "safety", "period_ticks": 1 },
      { "name": "log", "period_ticks": 10 }
    ]
  },
  "guidance": {
    "type": "PNG_3D",
    "navigation_constant": 4.0,
    "max_accel_mps2": 350.0,
    "max_accel_rate_mps3": 2000.0
  },
  "safety": {
    "sensor_timeout_s": 0.1,
    "command_hold_s": 0.2,
    "reject_nan": true,
    "reject_old_seq": true
  }
}
```

### 12.5 runtime.json 示例

```json
{
  "schema_version": 1,
  "campaign": {
    "campaign_id": "baseline_mc_001",
    "instance_count": 8,
    "schedule": "PARALLEL",
    "max_parallel_instances": 4,
    "base_random_seed": 12345,
    "failure_strategy": "CONTINUE_ON_FAILURE"
  },
  "network": {
    "protocol_version_major": 1,
    "protocol_version_minor": 0,
    "environment_base_port": 50000,
    "flight_control_base_port": 50001,
    "host": "127.0.0.1"
  },
  "logging": {
    "output_dir": "runs/baseline_mc_001",
    "instance_dir_template": "instance_${instance_id}",
    "binary_logs": true,
    "event_log": true,
    "flush_every_steps": 100
  },
  "instances": [
    {
      "instance_id": 0,
      "scenario": "configs/baseline/scenario.json",
      "flight_control": "configs/baseline/flight_control.json",
      "faults": "configs/baseline/faults.json",
      "random_seed": 12345
    },
    {
      "instance_id": 1,
      "scenario": "configs/baseline/scenario.json",
      "flight_control": "configs/baseline/flight_control.json",
      "faults": "configs/baseline/faults.json",
      "random_seed": 12346
    }
  ],
  "tools": {
    "environment_program": "./build/environment_sim/environment_sim",
    "flight_control_program": "./build/flight_control_sim/flight_control_sim",
    "write_summary_json": true,
    "write_run_manifest": true,
    "write_campaign_summary": true
  }
}
```

### 12.6 faults.json 示例

```json
{
  "schema_version": 1,
  "faults": [
    {
      "time_s": 12.5,
      "duration_s": 3.0,
      "target": "sensor.seeker.los_rate",
      "type": "BIAS",
      "value": 0.001
    }
  ]
}
```

### 12.7 配置校验规则

配置加载后必须执行校验：

```text
1. schema_version 必须支持。
2. 必填字段缺失则启动失败。
3. 数值字段必须为 finite。
4. 频率、步长、质量、缓存数量等字段必须在允许范围内。
5. 枚举字符串必须能映射到内部枚举。
6. 路径字段必须存在或符合创建策略。
7. 未识别字段默认允许，但必须记录 warning，便于兼容扩展。
8. 多实例配置中 `instance_id` 必须唯一。
9. 自动分配或显式配置的端口不能冲突。
10. 每个实例的日志目录不能冲突。
11. `instance_count` 必须与 `instances` 数量一致，或明确允许自动生成实例。
```

配置错误不允许静默使用默认值。只有文档明确标记为可选的字段才允许默认值。

### 12.8 配置清单

每次运行生成：

```text
run_manifest.json
```

内容包括：

```text
campaign_id
instance_id
program_version
git_commit
build_time
compiler
protocol_version
config_file_list
config_crc32
random_seed
run_mode
aero_table_path
aero_table_file_version
aero_table_extrapolation_policy_source
aero_table_height_min_m / aero_table_height_max_m
aero_table_actuator_min_rad / aero_table_actuator_max_rad
terrain_missing_policy
terrain_tile_paths
start_time_wall_clock
env_port
fc_port
```

当前实现的 `run_manifest.json` 已记录软件/Git/编译器/构建时间、工作树状态、协议版本、
配置路径、配置文件列表及逐文件 CRC32、运行目录、标准日志路径、端口、步长、运行模式、
实例随机种子、v1/v2 气动表启用状态、路径、内部文件格式版本、
气动表包络外策略来源/覆盖值、v1 可选包线元数据、v2 六轴维度、
地形启用状态、LOS 遮挡开关、缺瓦片策略、
平坦填充高度和加载的地形瓦片路径列表；可选 surrogate 模型路径、模型版本和训练数据版本
及 Mach/迎角/侧滑角适用包线也会进入 manifest。运行目录同时保存 scenario、runtime、faults
三个实际输入字节快照，避免源配置后续变化破坏复现链。

## 13. 日志与回放

### 13.1 日志类型

多实例仿真时，日志必须按实例隔离：

```text
runs/<campaign_id>/
  campaign_summary.json
  instance_0000/
    run_manifest.json
    config_snapshot_scenario.json
    config_snapshot_runtime.json
    config_snapshot_faults.json
    trajectory.csv
    sensor_log.bin
    command_log.bin
    fc_internal_log.bin
    trajectory_diagnostics.csv
    event_log.txt
    summary.json
  instance_0001/
    ...
```

单个实例内的日志类型：

```text
trajectory.csv
  环境真值状态和动力学诊断量的逐步 CSV 记录

sensor_log.bin
  环境发送给飞控的传感器帧

command_log.bin
  飞控发送给环境的控制指令

fc_internal_log.bin
  飞控内部状态、制导输出、保护动作

trajectory_diagnostics.csv
  数值健康、模型包线、积分器和气动模型状态

event_log.txt
  人可读事件日志

summary.json
  单次仿真摘要
```

`campaign_summary.json` 由 `instance_manager` 汇总生成，记录全部实例的运行状态、脱靶量统计、故障统计和失败原因。

诊断日志至少记录：

```text
sim_time
quat_norm_error
dcm_orthogonality_error
mass_kg
propellant_mass_kg
inertia_min_eigenvalue
aero_model_flags
aero_uncertainty_scale
integrator_type
dt_s
max_force_norm
max_moment_norm
```

`aero_model_flags` 用于标记气动表外推、钳位、上一帧保持、代理模型失效或缺失数据。
`model_degradation_flags` 汇总气动告警、地形查询告警、质量/惯量无效和姿态数值误差，
并以 `model_degradation_flags_or` 进入 `summary.json`、`campaign_summary.json` 和
`batch_stats`。
该字段必须进入单实例摘要和批量统计，避免模型越界在回归中被命中结果掩盖。

### 13.2 关键指标

仿真摘要至少包括：

```text
hit_flag
miss_distance
time_of_closest_approach
max_command_norm
max_actual_accel
sensor_dropout_count
command_timeout_count
fault_count
simulation_steps
exit_reason
```

批次摘要至少包括：

```text
campaign_id
instance_count
completed_count
failed_count
hit_count
miss_count
timeout_count
hit_rate
miss_distance_min
miss_distance_max
miss_distance_mean
miss_distance_std
failed_instances
```

`hit_count`、`miss_count` 和 `timeout_count` 对有摘要的实例互斥，三者之和必须等于
`summary_available_count`；进程失败且没有摘要的实例只进入 `failed_count`/`failed_instances`。

脱靶量：

$$
d_{\min}
=
\min_k
\left\|
\mathbf p_t(k) - \mathbf p_m(k)
\right\|
$$

### 13.3 回放一致性

回放工具必须支持：

```text
1. 只回放日志，不重新计算。
2. 用 sensor_log 重新驱动飞控。
3. 对比两次飞控输出差异。
4. 对比两次环境真值差异。
```

P8 工具链按职责拆分：

```text
tools/replay
  输入 sensor_log.bin、flight_control.json、instance_id。
  重新驱动飞控静态库或 flight_control_sim，生成 replayed_command_log.bin。

tools/compare_logs
  输入两个 sensor_log.bin、command_log.bin 或 trajectory.csv。
  协议日志按帧校验序号、时间戳、模式、状态位和浮点字段；轨迹按表头和数值行比较。
  输出首个发散帧、最大差异和阈值判定。

tools/batch_stats
  输入多个 summary.json 或 campaign_summary.json。
  输出命中率、脱靶量均值/标准差、失败原因分布、故障影响统计、
  数值诊断采样数、最大四元数范数误差、最大 DCM 正交性误差、最小质量、
  最小惯量、模型降级 flags 按位或、气动 flags 按位或、气动外推采样数和
  wall-clock 性能统计。

tools/batch_runner
  输入手写运行清单并顺序调用 instance_manager，或从 runtime 模板确定性展开 Monte Carlo
  样本清单。模板占位符包括 `${sample_index}`、`${random_seed}`、
  `${sample_output_dir}`、`${uniform:stream:min:max}` 形式的确定性均匀扰动，
  `${lhs_uniform:stream:min:max}` 形式的确定性 LHS 均匀分层扰动，
  `${halton_uniform:base:min:max}` 形式的确定性 Halton 低差异均匀扰动，
  `${normal:stream:mean:stddev}` 形式的确定性正态扰动、
  `${lognormal:stream:mu:sigma}` 形式的确定性对数正态扰动、
  `${truncated_normal:stream:mean:stddev:min:max}` 形式的确定性截断正态扰动、
  `${choice:stream:option|option}` 形式的确定性离散选择，以及
  `${correlated_normal:stream:base_stream:mean:stddev:rho}` 形式的确定性相关正态扰动。
```

比较工具显式区分两种模式：绝对/相对容差都为零时执行协议日志逐字节或轨迹行文本
精确比较；任一容差非零时执行浮点绝对/相对阈值比较。协议字段、状态位、实例号、
序号和模式字段在两种模式下都必须精确一致。比较结果至少包含：

```text
frame_count_left
frame_count_right
first_divergent_frame
max_abs_error
max_rel_error
status_bit_mismatch_count
mode_mismatch_count
comparison_mode
verdict
```

固定随机种子回归应同时保留两类基准：

```text
byte_exact_baseline
  适用于协议日志、固定平台和确定性模块。

tolerance_baseline
  适用于跨平台浮点、气动表/代理模型和长时间积分。
```

## 14. 故障注入

### 14.1 传感器故障

```text
SENSOR_FAULT_BIAS
SENSOR_FAULT_DRIFT
SENSOR_FAULT_RAMP_BIAS
SENSOR_FAULT_NOISE_INCREASE
SENSOR_FAULT_DROPOUT
SENSOR_FAULT_STUCK
SENSOR_FAULT_DELAY
SENSOR_FAULT_SATURATION
```

### 14.2 执行机构故障

```text
ACTUATOR_FAULT_STUCK
ACTUATOR_FAULT_BIAS
ACTUATOR_FAULT_RATE_LIMIT
ACTUATOR_FAULT_DELAY
ACTUATOR_FAULT_DISABLED
```

### 14.3 通信故障

```text
COMM_FAULT_DROP_PACKET
COMM_FAULT_DELAY_PACKET
COMM_FAULT_DUPLICATE_PACKET
COMM_FAULT_CORRUPT_PACKET
COMM_FAULT_REORDER_PACKET
```

当前实现对上述矩阵采用独立语义，而不是把所有故障折叠为“清有效位”：

- `SENSOR_FAULT_NOISE_INCREASE` 使用实例私有确定性随机流；`value`/`value_xyz` 是附加
  白噪声标准差。
- `SENSOR_FAULT_STUCK` 在故障起点捕获实际测量并保持；`SENSOR_FAULT_DELAY` 的 `value`
  是 1 到 16 的动态延迟步数；`SENSOR_FAULT_SATURATION` 使用 `min_value`/`max_value`。
- 执行机构速率/行程退化使用 `scale`，延迟使用步数，失能把驱动命令置为中立值；所有
  参数只在故障激活步临时作用，恢复后回到配置基线。
- 显式 `COMM_FAULT_DROP_PACKET`、`COMM_FAULT_DUPLICATE_PACKET` 和
  `COMM_FAULT_CORRUPT_PACKET` 操作实际 UDP 发送；损坏报文保留旧 CRC 以验证接收端拒绝。
  `LOCKSTEP` 下真实丢包/损坏可导致控制超时，恢复场景应使用 `FREE_RUNNING`。

### 14.4 故障脚本

故障由配置脚本指定：

```text
time = 12.5
target = sensor.seeker.los_rate
fault = bias
value = 0.001
duration = 3.0
```

## 15. 验证与确认

### 15.1 测试层级

```text
L0 静态检查
  编译警告、格式、静态分析

L1 单元测试
  common、飞控模块、环境模块

L2 模块集成测试
  传感器链路、执行机构链路、制导链路

L3 双进程 SIL 测试
  environment_sim + flight_control_sim

L4 批量 Monte Carlo
  随机初值、随机噪声、随机故障

L5 回归测试
  固定场景、固定随机种子、固定输出基准

L6 数值与包线审计
  积分器误差趋势、四元数/惯量不变量、气动模型包线和外推报警
```

### 15.2 单元测试要求

common：

```text
vec3_dot
vec3_cross
vec3_norm
quat_normalize
quat_to_dcm
packet_crc
config_parse
ring_buffer_delay
```

飞控：

```text
三维比例导引方向正确
闭合速度符号正确
指令范数限幅正确
变化率限制正确
传感器超时进入保护
旧帧被拒绝
NaN 输入被拒绝
```

环境：

```text
六自由度状态积分接口正确
四元数范数保持在容差内
DCM 正交性保持在容差内
质量为正、惯量正定
执行机构一阶响应正确
传感器延迟正确
导引头相对量正确
目标机动模型正确
故障注入按时触发
命中/脱靶判定正确
气动模型边界输入有限且方向合理
气动表外推按策略报警或拒绝
```

### 15.3 回归判据

固定种子场景下，关键输出必须满足容差：

$$
\left|
d_{\min}^{\text{new}}
-
d_{\min}^{\text{ref}}
\right|
\le
\epsilon_d
$$

控制指令差异：

$$
\max_k
\left\|
\mathbf u_k^{\text{new}}
-
\mathbf u_k^{\text{ref}}
\right\|
\le
\epsilon_u
$$

数值健康判据：

$$
\left|
\left\|\mathbf q_{BI}\right\| - 1
\right|
\le
\epsilon_q
$$

$$
\left\|
\mathbf C\mathbf C^\mathsf{T} - \mathbf I
\right\|_F
\le
\epsilon_C
$$

其中 $\epsilon_q$ 和 $\epsilon_C$ 由回归配置给出。若质量非正、惯量非正定、
气动模型包线错误、协议状态位不一致或实例输出目录冲突，回归必须失败，不允许只按
命中结果判定通过。

### 15.4 新增模型的验证门槛

新增气动表或代理模型必须先通过离线测试，再接入闭环主链路。当前线性代理模型已覆盖
固定格式加载、缺项拒绝、Mach/alpha/beta 线性推理和气动力主路径单测：

```text
1. 配置和数据文件校验：维度、单位、单调网格、缺失值、CRC 或版本。
2. 插值/推理测试：网格点精确、单元内部连续、边界有限，或代理输出有限且可解释。
3. 包线测试：低于/高于范围时执行 ERROR、CLAMP_AND_WARN 或 HOLD_LAST_VALID。
4. 方向测试：阻力方向与相对速度相反，控制面偏转产生的力矩符号可解释。
5. 闭环回归：同一随机种子下命令日志和摘要满足阈值。
```

新增回放或批量统计工具必须覆盖：

```text
1. 正常日志。
2. 截断日志。
3. CRC 错误日志。
4. 实例号不匹配日志。
5. 帧数不一致日志。
6. 首帧和中间帧发散定位。
```

容差由测试配置指定。

### 15.5 从架构依据到定量可信的验证阶梯

公开文献只能证明本项目的架构路线具有研究合理性，不能直接证明当前参数或模型有真实精度。
因此验证必须按以下阶梯推进：

```text
V0 代码和配置可构建
  -> 严格编译、JSON schema_version、必填节和路径校验

V1 模块数学正确
  -> 向量/矩阵/四元数、协议 CRC、地理坐标、插值和随机数固定种子测试

V2 解析算例和不变量正确
  -> 真空抛体、恒力/恒力矩、质量/惯量正性、四元数范数、DCM 正交性

V3 双进程 SIL 因果闭环正确
  -> 飞控只读 SensorFrame，环境只消费 ControlCommand，一帧测量对应一帧命令

V4 可复现和可回放
  -> 固定种子双跑一致，sensor_log 可回放，command_log 可容差比较

V5 批量和故障覆盖
  -> 多实例隔离、Monte Carlo、故障触发/恢复、失败路径和统计汇总

V6 数据标定和外部交叉验证
  -> DEM/气动/传感器/执行机构真实或公开基准数据，独立仿真器或实验数据对照

V7 HIL/实测验证
  -> 半实物接口、真实硬件时序、台架或飞行数据；当前项目默认不进入该层
```

当前仓库主要覆盖 V0 到 V5。V6 之后属于模型可信度和工程验证扩展，不能由单元测试、
闭环测试或文献引用替代。设计评审、论文写作或项目申请书中必须明确这一区分：

- 文献依据回答“为什么这样组织仿真系统是合理的”。
- 代码和测试回答“当前实现是否满足仓库约定并可重复运行”。
- 数据标定和外部验证才回答“模型是否定量接近真实对象”。

## 16. C 代码工程规范

### 16.1 编码规则

建议采用接近 MISRA-C 的约束：

- 不使用隐式函数声明。
- 不使用未初始化变量。
- 不在主循环中动态分配内存。
- 不在业务代码中直接调用 `exit`。
- 不忽略函数返回值。
- 所有数组访问必须有边界检查。
- 所有网络输入必须校验长度和版本。
- 所有浮点输入必须检查 `isfinite`。

### 16.2 错误处理

统一返回码：

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

### 16.3 模块接口要求

每个模块至少提供：

```text
init
reset
step/update
get_status
shutdown
```

例如：

```c
SimStatus sensor_seeker_init(SeekerSensor *sensor, const SeekerConfig *cfg);
SimStatus sensor_seeker_update(SeekerSensor *sensor, const WorldState *world, SensorFrame *frame);
SimStatus sensor_seeker_get_status(const SeekerSensor *sensor, uint32_t *status);
```

## 17. 性能与实时性

### 17.1 性能指标

工业级仿真需要记录：

```text
平均步耗时
最大步耗时
网络收发耗时
飞控计算耗时
日志写入耗时
丢帧数量
超时数量
```

### 17.2 实时裕度

实时模式下：

$$
T_{\text{step,max}}
<
\Delta t
$$

建议记录实时裕度：

$$
M_{\text{rt}}
=
\Delta t - T_{\text{step,max}}
$$

如果 $M_{\text{rt}} < 0$，系统应记录实时性违例事件。

## 18. 开发交付阶段

### 18.1 P0 工程底座

交付：

- CMake 工程。
- common 数学库。
- 协议结构。
- 日志框架。
- 配置读取。
- 单元测试框架。

### 18.2 P1 双进程闭环

交付：

- `environment_sim` 独立进程。
- `flight_control_sim` 独立进程。
- 二进制 UDP 协议。
- SensorFrame/ControlCommand ICD。
- 锁步仿真。
- 基础日志。

### 18.3 P2 三维制导与传感器误差

交付：

- 三维弹目相对测量。
- 三维比例导引。
- 传感器噪声、延迟、丢包。
- 飞控保护状态机。
- 回放工具。

### 18.4 P3 六自由度环境接口

交付：

- 六自由度状态。
- 四元数姿态。
- 力/力矩模型接口。
- 执行机构模型。
- RK4 积分器。
- 模型配置文件。

### 18.5 P4 批量验证与故障注入

交付：

- Monte Carlo 批量运行。
- 故障注入脚本。
- 回归测试基准。
- summary 统计报告。
- 性能和实时性统计。

## 19. 与参考论文的对应关系

本节不把论文当作“当前实现已经高保真”的证明，而是记录研究依据如何约束设计。
完整文献清单、PDF 来源和 GB/T 7714 著录见
[`../references/REFERENCES_GBT7714.md`](../references/REFERENCES_GBT7714.md)；
正文级论证见 [research_report_evidence_basis.md](research_report_evidence_basis.md)。

| 设计主张 | 研究依据 | 对设计的约束 | 当前实现状态 |
|---|---|---|---|
| 飞控与环境应分离为 SIL 闭环 | Prado 等的控制器/被控对象软件通过 Ethernet/UDP 交换信息 | 飞控进程不得读取环境真值；环境只发布 `SensorFrame`，只消费 `ControlCommand` | 双进程 UDP LOCKSTEP、协议 CRC、实例号校验和闭环测试已实现 |
| 仿真平台不只是动力学方程，还需要调度、输入和记录 | NASA Trick 把时间调度、输入文件、数据记录和运行控制作为仿真基础设施 | 仿真时间、配置、日志、manifest、回放和批处理是设计一等能力 | `sim_time`、配置、日志、`replay`、`compare_logs`、`batch_runner` 已实现轻量版本 |
| 环境被控对象应按力/矩和 6DOF 状态推进 | Marzouk 6DOF 飞行动力学方程组织 | 环境主链必须包含力/矩、质量/惯量、姿态四元数和积分器，而不是直接改位置 | 6DOF 状态、Euler/RK2/RK4、重力/大气/气动/推进力链已接入 |
| 数值收敛不等于动力学可信 | Sharma 等指出 6DOF 转录可能出现四元数不变量漂移和动态不可行 | 回归不能只看命中结果，必须输出四元数、DCM、质量/惯量和积分诊断 | `trajectory_diagnostics.csv`、summary/campaign 诊断聚合已实现 |
| 气动代理必须有数据、包线和外推评估 | AirfRANS 用高保真 CFD 数据和外推任务评估代理模型 | 气动表和 surrogate 必须记录来源、版本、包线、外推状态和不确定度占位 | v1/v2 气动表、线性 surrogate、manifest 和气动 flags 已接入；真实数据未接入 |
| 轨迹研究需要批处理和参数扫描 | Dymos/OpenMDAO 支持多阶段、参数化和优化研究 | 项目应支持多实例、Monte Carlo、批量统计和容差比较 | `instance_manager`、`batch_runner`、`batch_stats` 和 128 实例 runtime 已具备 |
| 制导控制研究应覆盖测量、估计、制导、控制和保护链 | 中文学位论文与 Kamath 等 6DOF 制导研究 | 飞控不能只有 PNG 输出；需有估计、状态机、自动驾驶仪、控制分配和安全保护 | 飞控静态库、PNG、估计、模式、自动驾驶仪、控制分配和保护测试已接入 |

中文学位论文主要用于参考导弹制导控制仿真的模块组织、坐标/相对运动、比例导引、
姿态控制扩展方向、视景/轨迹记录和仿真系统组织方式。它们不提供本项目的真实型号参数。

Prado 等、Trick、Marzouk、AirfRANS、Dymos、Kamath 和 Sharma 等文献分别提供
SIL 架构、仿真基础设施、6DOF 建模、气动代理数据、批量/优化工具、6DOF 制导约束和
数值失效模式方面的依据。它们共同支持本设计的工程路线，但不能替代本项目自己的构建、
测试、标定和外部验证。

本设计按工程化仿真软件组织，不直接实现真实型号参数、真实硬件接口或半实物硬件闭环。
任何对外表述应使用“软件在环闭环仿真研究原型”“可回放、可批量、可扩展的工程骨架”
等限定语，避免宣称武器级、适航级或实飞确认的高保真能力。

## 20. 当前实现建议

虽然设计按工业级目标展开，但编码仍应按工程底座顺序交付：

```text
1. 建 CMake + common。
2. 实现协议、日志、配置和数学库。
3. 实现 environment_sim 与 flight_control_sim 双进程最小闭环。
4. 接入三维比例导引。
5. 接入传感器误差和执行机构模型。
6. 接入六自由度状态接口。
7. 做批量测试、故障注入和回放。
```

重点不是先写一个简化演示，而是从第一天就按最终工业级架构拆模块、定协议、建日志、做测试。模型保真度可以逐步填充，但接口、数据流、状态机和验证体系必须一次设计到位。

## 21. 设计完成基线与落地边界

本节作为当前阶段的设计收敛基线，用于把目标设计、当前代码和后续实现计划对齐。
后续开发必须优先保持本节定义的边界，避免把临时模型误描述为最终能力。

### 21.1 当前已经落地的闭环设计

当前工程已经落地的软件在环闭环为：

```text
environment_sim
  -> SensorFrame
  -> flight_control_sim
  -> ControlCommand
  -> environment_sim
```

单个飞行实例的定义保持不变：

```text
FlightInstance(i) =
  environment_sim(instance_id=i)
  + flight_control_sim(instance_id=i)
```

每个实例只允许访问本实例的运行状态、端口、日志目录和随机流。多个实例可以
共享只读程序、默认配置和地图资源，但不得共享可变状态。

环境始终拥有仿真时间。确定性回归采用 `LOCKSTEP`，实时演示和网络恢复可采用
`FREE_RUNNING`；两者都按固定仿真步推进，墙钟不会反馈到物理状态：

$$
t_{k+1}=t_k+\Delta t
$$

每一个有效仿真步满足：

$$
\text{SensorFrame}_k
\rightarrow
\text{ControlCommand}_k
\rightarrow
\mathbf x_{k+1}
$$

其中 \(\mathbf x_k\) 是环境程序内部的真值状态，飞控程序不能直接读取该状态，
只能读取协议中的传感器测量。

`SIL_REALTIME` 使用单调绝对墙钟节拍，并把睡眠、超限、控制往返和实时裕度写入
`performance.json`/`fc_performance.json`；`SIL_FAST` 不等待墙钟。正常结束由环境发送
带 CRC 的仿真停止控制帧，飞控无需依赖接收超时退出。

### 21.2 环境被控对象设计基线

环境程序的被控对象按六自由度刚体设计。连续状态为：

$$
\mathbf x =
\left[
\mathbf r_e,\,
\mathbf v_e,\,
\mathbf q_{be},\,
\boldsymbol\omega_b
\right]
$$

其中：

- \(\mathbf r_e\)：ECEF 位置，单位 m。
- \(\mathbf v_e\)：ECEF 速度，单位 m/s。
- \(\mathbf q_{be}\)：机体系到 ECEF 的姿态四元数。
- \(\boldsymbol\omega_b\)：机体系角速度，单位 rad/s。

平动方程设计为：

$$
\dot{\mathbf r}_e = \mathbf v_e
$$

$$
\dot{\mathbf v}_e =
\frac{1}{m}\mathbf C_{be}\mathbf F_b
+ \mathbf g_e
+ \mathbf a_{\text{rot},e}
$$

其中：

$$
\mathbf a_{\text{rot},e}
=
-2\boldsymbol\Omega_e \times \mathbf v_e
-
\boldsymbol\Omega_e \times
\left(
\boldsymbol\Omega_e \times \mathbf r_e
\right)
$$

转动方程设计为：

$$
\dot{\mathbf q}_{be}
=
\frac{1}{2}
\mathbf q_{be}
\otimes
\left[
0,\boldsymbol\omega_b
\right]
$$

$$
\dot{\boldsymbol\omega}_b
=
\mathbf I_b^{-1}
\left(
\mathbf M_b
-
\boldsymbol\omega_b
\times
\mathbf I_b\boldsymbol\omega_b
\right)
$$

环境力链的设计顺序为：

```text
scenario/runtime
  -> WGS-84 / terrain / target truth
  -> atmosphere / gravity / propulsion / aerodynamics / mass
  -> actuator response
  -> force_b / moment_b
  -> 6DOF integrator
  -> geodetic state / AGL / collision / hit detect
  -> sensor models
  -> SensorFrame
```

当前代码已经接入该主链路，但仍保留以下工程边界：

- 飞控输出仍解释为 ECEF 加速度级虚拟指令，再换算为等效机体系控制力。
- 气动模型默认 baseline 为可配置低阶模型；配置 `aerodynamics.table_path`
  时可加载版本化 v1 气动表进入统一力模型。v1 表执行 Mach/AoA/beta 插值，
  高度/舵偏可作为包线元数据进入 manifest。配置 `aerodynamics.table_v2_path` 时，
  使用 Mach/AoA/beta/高度/俯仰舵偏/偏航舵偏六维规则网格和 64 角多线性插值，
  v2 舵效不再叠加低阶线性项。v1/v2 均覆盖固定小端文件、CRC、单位和包络策略。
- 可选质量属性模型、风切变、正弦阵风和固定种子一阶高斯-马尔可夫湍流已进入主链路。
- 目标模型已区分匀速与脚本 ECEF 加速度段，并对步内机动边界分段积分；最近点和命中
  使用步间连续相对线段判定。
- 数值诊断、模型降级 flags 和气动 flags 已写入 `trajectory_diagnostics.csv`，并聚合到
  `summary.json`、`campaign_summary.json` 和 `batch_stats`。

因此当前环境程序可用于闭环结构、数值积分、坐标系统、传感器接口和多实例验证；
控制分配、真实来源最小 DEM 和工程故障链已经覆盖，但没有目标外形真实气动、器件标定
和 V6/V7 外部验证，仍不能描述为高保真型号仿真。

当前已达到的环境工程基线为：

```text
低阶模型可运行
  -> 表格气动可校验
  -> 包线越界可报警
  -> 数值不变量可审计
  -> 批量统计可回归
```

气动表和代理模型都必须保持只读、确定性和实例隔离。代理模型不作为 P5/P6 正确性
前置条件；当前已接入最小线性只读推理，复杂代理模型仍属于 P8 之后的模型保真度扩展，
必须先通过表格模型同等级的包线和回放验证。

### 21.3 传感器设计基线

传感器模型不得直接把理想真值复制给飞控。每类传感器按以下通用链路设计：

```text
truth measurement
  -> bias
  -> scale/noise/random walk
  -> quantization/limit
  -> sample-and-hold
  -> fixed delay
  -> dropout/fault flag
  -> SensorFrame
```

标量测量的通用形式为：

$$
y_k =
\operatorname{clip}
\left(
\operatorname{quantize}
\left(
y_k^\ast + b_k + n_k
\right)
\right)
$$

随机游走偏置按实例私有随机流更新：

$$
b_{k+1}=b_k+\sigma_{\text{rw}}\sqrt{\Delta t}\,w_k
$$

三轴测量按分量执行同样处理，并在导引头 LOS 单位向量测量后重新归一化：

$$
\hat{\mathbf r}_{\text{meas}}
=
\frac{\hat{\mathbf r}_{\text{raw}}}
{\left\|\hat{\mathbf r}_{\text{raw}}\right\|}
$$

当前已经进入主链路的传感器包括：

- IMU 陀螺仪：\(\boldsymbol\omega_b\)。
- 加速度计：ECEF 加速度测量。
- 速度计：ECEF 速度测量。
- 导引头：距离、LOS 单位向量、LOS 角速度和闭合速度。
- 大地坐标、高度和 AGL：当前作为派生地理测量直接写入帧。

基础 `faults.json` 已接入环境主链路，覆盖第 14 节的传感器、执行机构和通信故障矩阵，
包括确定性噪声增大、起点捕获卡滞、逐测量动态延迟、饱和、执行机构速率/行程退化、
延迟/失能，以及实际 UDP 丢弃、重复和 CRC 损坏。恢复保持、周期突发、模式变延迟和
上一帧重放仍受支持。单实例和成功批次会汇总故障触发次数和影响步数。
当前协议 v1 只有帧级时间戳；虽然 `FREE_RUNNING` 已覆盖基础丢包/重复/损坏恢复，
每传感器独立时间戳、时钟漂移和操作系统网络栈时序相关性仍需要协议 v2 与 V6/V7 外部验证。

### 21.4 飞控程序设计基线

飞控程序的最终工程链路为：

```text
SensorFrame
  -> interface validation
  -> scheduler
  -> navigation / estimator
  -> health and mode state machine
  -> guidance manager
  -> autopilot / command manager
  -> safety monitor
  -> ControlCommand
```

当前已经落地的是协议校验、安全监视、导航估计、模式状态机、三维比例导引、
虚拟自动驾驶仪、命令管理、幅值限幅和变化率限幅。
比例导引指令为：

$$
\mathbf a_c =
N V_c
\left(
\boldsymbol\omega_{\text{LOS}}
\times
\hat{\mathbf r}
\right)
$$

幅值限幅为：

$$
\mathbf a_{\text{cmd}} =
\begin{cases}
\mathbf a_c,
&
\left\|\mathbf a_c\right\|\le a_{\max}
\\
\dfrac{a_{\max}}{\left\|\mathbf a_c\right\|}
\mathbf a_c,
&
\left\|\mathbf a_c\right\|>a_{\max}
\end{cases}
$$

变化率限制已经进入 `command_manager` 主链路：

$$
\Delta \mathbf a =
\mathbf a_{\text{cmd},k}
-
\mathbf a_{\text{cmd},k-1}
$$

$$
\mathbf a_{\text{cmd},k}^{\text{limited}}
=
\mathbf a_{\text{cmd},k-1}
+
\operatorname{sat}_{\dot a_{\max}\Delta t}
\left(
\Delta \mathbf a
\right)
$$

状态机已经进入 `fc_modes` 主链路：

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

当前 `flight_control_sim` 已经是具备飞控任务链路的模拟件，姿态/角速度自动驾驶仪、
俯仰/偏航舵偏控制分配和按 `scheduler.tasks[]` 执行的多速率缓存更新已经进入主链路。
固定记录格式内部日志、CSV 解码和阶跃/反向/丢包恢复控制品质报告已经补齐。
它仍不是真实弹载飞控软件；型号带宽、稳定裕度和飞行品质必须由 V6/V7 数据与试验确认。

### 21.5 多实例与并行化设计基线

多实例运行的工程目标是实例级并行，而不是在一个环境进程内部混合多个对象：

```text
instance_manager
  -> environment_sim(i), flight_control_sim(i)
  -> environment_sim(j), flight_control_sim(j)
  -> ...
```

实例端口分配为：

$$
P_{\text{env}}(i)=P_{\text{env,base}}+2i
$$

$$
P_{\text{fc}}(i)=P_{\text{fc,base}}+2i
$$

当前已经实现进程级并发、串行调度、端口隔离、输出隔离，并且管理器会读取
`runtime.json` 的 `instances[]`，包括：

- `instance_id`
- `scenario`
- `flight_control`
- `faults`
- `random_seed`
- `enabled`

管理器还会读取 `runtime.tools.environment_program` 和
`runtime.tools.flight_control_program`，用于覆盖默认子程序路径，避免编排工具
必须从仓库根目录启动。

应用层就绪端口分配会避开整个批次所有环境/飞控业务端口。管理器集成测试覆盖
`PARALLEL`、`SEQUENTIAL`、`CONTINUE_ON_FAILURE` 和 `STOP_ON_FAILURE`，并验证
失败实例列表和后续实例继续/跳过语义。

关于 GPU 或 SIMD 并行化，当前代码没有实现 GPU 后端。设计上只保留如下边界：

- 单实例闭环语义不能因并行化改变。
- 实例之间仍不得共享可变状态。
- 可并行化对象优先选择批量运行中的独立实例、传感器批处理、气动表插值和统计后处理。
- GPU 加速属于 P8 之后的性能扩展，不作为当前 P5-P7 正确性验收条件。

### 21.6 工程计划完成判据

P5 完成判据：

- `faults.json` 被环境程序读取和校验。当前基础能力已实现。
- 故障按仿真时间触发，能作用于传感器有效位、测量值或执行机构。当前基础能力已实现。
- 触发、持续、恢复和拒绝原因写入 `event_log.txt`。当前开始/恢复事件已实现。
- `summary.json` 和成功实例的 `campaign_summary.json` 汇总故障统计。当前基础能力已实现。
- 闭环测试覆盖延迟预热、丢包、通信帧延迟、上一帧重放乱序、至少一种脚本故障和固定种子双跑一致性。当前基础能力已实现。
- 周期突发、模式变延迟、真实来源 DEM LOS、质量惯量演化和复杂风模型已有确定性回归。
- 环境诊断日志覆盖四元数范数、DCM 正交性、质量/惯量有效性和气动包线状态。
- 气动表模型接入时，必须覆盖插值、边界、外推策略、文件校验和闭环回归。

P6 完成判据：

- 飞控模块形成可测试静态库。当前已实现。
- PNG 有独立单元测试，覆盖 LOS 方向、加速度限幅、负闭合速度和 NaN 距离拒绝。
  当前已完成覆盖。
- 加速度变化率限制接入主链路。当前已实现。
- 状态机、命令保持、传感器超时和 NaN/Inf 保护有单元测试或闭环回归。当前已完成覆盖。
- 姿态/角速度自动驾驶仪、执行机构/舵面控制分配和多速率任务执行当前已进入主链路；
  单测覆盖内环开启分配路径、内环关闭的加速度透传路径和连续帧限幅/速率/舵偏边界。
- 固定内部日志格式及 CSV 解码已实现；标准机动报告覆盖阶跃、指令反向、丢包恢复和版本化阈值。
- P6 工程验收已完成。真实型号控制品质属于 V6/V7，不能用 SIL 报告替代。

P7 完成判据：

- 管理器读取逐实例配置，而不是硬编码 baseline 路径。当前已实现。
- 支持并验证 `PARALLEL`、`SEQUENTIAL`、并发上限和失败策略。两种调度模式及两种失败策略均已由多实例集成测试覆盖。
- 单实例失败不会终止其他实例。`CONTINUE_ON_FAILURE` 已验证首实例飞控配置失败后第二实例仍完成；`STOP_ON_FAILURE` 已验证后续实例被明确标记跳过。
- `campaign_summary.json` 汇总每个实例的退出原因、最小距离、端口、配置路径、随机种子和故障统计。当前已实现。
- 子程序路径可通过 `runtime.tools` 配置。当前已实现，并由管理器集成测试覆盖。
- 端口占用预检失败路径已由管理器集成测试覆盖。
- 应用层就绪/心跳握手当前已实现，飞控完成业务初始化后发送 `PACKET_HEARTBEAT`，管理器收到合法心跳后才启动环境。
- P7 计划内主链路能力当前已完成。

P8 完成判据：

- `sensor_log.bin` 可回放驱动飞控。
- `command_log.bin` 和 `fc_internal_log.bin` 可转换为可读格式。当前均已支持 CSV。
- `compare_logs` 可比较 sensor/command 协议日志和 `trajectory.csv`，定位首个发散帧，
  并输出精确/容差模式、浮点阈值、模式和状态位差异。
- 批量运行能输出逐实例脱靶量统计、成功率、失败实例、失败原因分布和配置快照。
- 回放结果在固定种子下可重复。
- 批量统计必须聚合故障影响、气动包线越界、数值不变量异常和子进程失败原因。
  当前已聚合故障影响、基础数值不变量、模型降级 flags 和气动 flags。
- 压力回归分为默认 short 4 实例/并发 2、可选 medium 16/4 和 long 128/8，
  均覆盖完成/失败计数、种子、诊断和 wall-clock 字段。
- 回归配置区分逐字节基准和版本化绝对/相对容差基准。当前已实现。
- `run_manifest.json` 记录 Git/构建身份、完整配置 CRC32、输入字节快照、运行模式、
  标准日志路径、资源版本和模型包线。当前已实现并进入闭环回归。
- `REPLAY_PASSIVE`、`REPLAY_WITH_FC` 和 `MONTE_CARLO` 工具工作流也会生成 sidecar
  run manifest，记录输入/输出 CRC、帧或样本数量、种子来源和结果状态。
- P8 工程验收已完成；目标机器真实长时容量仍属于外部 V6 基准。

P8 之后模型保真度扩展判据：

- `AERO_TABLE` v1/v2 具备版本化数据格式、CRC/单位校验、三维/六维插值测试、外推报警和闭环接入。
- `AERO_SURROGATE` 只允许离线训练、在线只读推理，并在 `run_manifest.json`
  中记录模型版本、训练数据版本和适用包线；在线推理会拒绝适用包线外输入。当前已实现。
- 真实 DEM 加载链有缺瓦片策略、真实来源 DEM LOS 遮挡闭环和地图预处理回归；当前已实现。

### 21.7 当前基线的证据边界

当前 P0-P8 仓库内软件验收入口已经闭合，表示 V0-V5 模块、主链路和测试工具具备自动证据；
这不是产品或型号完成百分比，也不表示物理模型已经达到真实对象精度或 V6/V7。按照
`research_report_evidence_basis.md` 的证据分级，当前设计基线
可支持以下结论：

- 双进程 SIL 闭环架构有公开研究中的同构案例支持，且仓库中已实现。
- 6DOF 力/矩推进、四元数姿态、不变量诊断和积分器对照是合理的飞行仿真方法路线。
- 日志、回放、批处理、固定随机种子和诊断聚合符合可复现实验平台的基本要求。
- 气动表和 surrogate 的接口路线合理，但真实气动可信度取决于后续数据来源和包线验证。

当前设计基线不能支持以下结论：

- 当前低阶气动、传感器、执行机构或控制律已经代表真实型号。
- 现有命中/脱靶结果可作为实物性能结论。
- 文献引用可以替代 CFD、风洞、台架、半实物或实测数据验证。
- 本项目已经等价于 JSBSim、Trick、Aerospace Blockset、STK 等成熟平台。

后续所有模型保真度扩展都必须在 `run_manifest.json`、`summary.json`、批量统计和文档中
记录数据来源、适用包线、外推处理和验证层级。没有完成 V6/V7 级验证前，只能宣称
“研究原型”和“工程骨架”，不能宣称“真实型号仿真器”。
