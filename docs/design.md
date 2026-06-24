# 飞控与环境闭环仿真系统工业级设计文档

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
      fc_interface.h
      fc_modes.h
      fc_scheduler.h
      fc_state.h
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
      scenario.h
      world_state.h
      earth_model.h
      geo_coordinate.h
      map_tile.h
      terrain_model.h
      atmosphere_model.h
      gravity_model.h
      target_model.h
      missile_plant_6dof.h
      aero_model.h
      propulsion_model.h
      mass_model.h
      actuator_model.h
      sensor_imu.h
      sensor_accel.h
      sensor_speed.h
      sensor_seeker.h
      sensor_noise.h
      fault_injection.h
      hit_detect.h
      recorder.h
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

所有模式都必须生成运行清单 `run_manifest`，记录配置版本、随机种子、程序版本、协议版本和日志文件路径。

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
1. 根据经纬度加载地形瓦片。
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
tile_index.bin
  -> 全部瓦片的经纬度范围、文件偏移、分辨率、CRC

tile_*.bin
  -> 高程网格、地表分类、有效性掩码

map_manifest.json
  -> 原始数据来源、生成时间、坐标基准、分辨率、工具版本
```

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

### 6.13 数值积分器

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
    env_record_truth_log
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
    "tile_index": "data/maps/tile_index.bin",
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
    "reference_area_m2": 0.01,
    "reference_length_m": 1.0,
    "drag_coefficient": 0.1,
    "control_force_coefficient": 0.0,
    "control_moment_coefficient": 0.0
  },
  "target": {
    "model": "SCRIPTED",
    "initial_lla_deg_m": [30.02, 120.05, 1000.0],
    "initial_velocity_ecef_mps": [0.0, 0.0, 0.0]
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
start_time_wall_clock
env_port
fc_port
```

## 13. 日志与回放

### 13.1 日志类型

多实例仿真时，日志必须按实例隔离：

```text
runs/<campaign_id>/
  campaign_summary.json
  instance_0000/
    run_manifest.json
    truth_log.bin
    sensor_log.bin
    command_log.bin
    fc_internal_log.bin
    event_log.txt
    summary.json
  instance_0001/
    ...
```

单个实例内的日志类型：

```text
truth_log.bin
  环境真值状态

sensor_log.bin
  环境发送给飞控的传感器帧

command_log.bin
  飞控发送给环境的控制指令

fc_internal_log.bin
  飞控内部状态、制导输出、保护动作

event_log.txt
  人可读事件日志

summary.json
  单次仿真摘要
```

`campaign_summary.json` 由 `instance_manager` 汇总生成，记录全部实例的运行状态、脱靶量统计、故障统计和失败原因。

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

## 14. 故障注入

### 14.1 传感器故障

```text
SENSOR_FAULT_BIAS
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
执行机构一阶响应正确
传感器延迟正确
导引头相对量正确
目标机动模型正确
故障注入按时触发
命中/脱靶判定正确
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

容差由测试配置指定。

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

戴琪昕论文用于参考：

- 坐标系和六自由度建模框架。
- 弹目相对运动方程。
- 比例导引法。
- 自动驾驶仪/姿态控制扩展方向。
- 全弹道闭环仿真组织方式。

李军、付沂辰论文用于参考：

- 仿真系统模块划分。
- 参数接收和运动驱动模块。
- 轨迹记录与回放。
- 视景显示和人机交互。

本设计按工业级仿真软件组织，不直接实现真实型号参数或半实物硬件接口。

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

闭环时序采用环境拥有仿真时间的锁步模式：

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

当前代码已经接入该主链路，但仍保留两项工程边界：

- 飞控输出仍解释为 ECEF 加速度级虚拟指令，再换算为等效机体系控制力。
- 气动模型当前为可配置低阶模型，不是气动表插值模型。

因此当前环境程序可用于闭环结构、数值积分、坐标系统、传感器接口和多实例验证；
在气动表、控制面分配、真实 DEM 和完整故障链路完成前，不把它描述为高保真型号仿真。

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

基础 `faults.json` 已接入环境主链路，可按仿真时间触发传感器偏置、
传感器强制无效/丢包、虚拟执行机构卡滞和命令缩放，并记录开始/恢复事件。
单实例和成功批次会汇总故障触发次数和影响步数。尚未完成的是更完整的故障类型库、
真实 DEM 遮挡闭环场景和更多工程级故障恢复模式。

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

当前已经落地的是协议校验、旧帧拒绝、导引头有效位检查、三维比例导引和幅值限幅。
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

下一步必须补齐的飞控设计项为变化率限制：

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

以及状态机：

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

在这些模块完成前，`flight_control_sim` 是“比例导引控制器模拟件”，不是完整飞控软件。

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

当前已经实现进程级并发、端口隔离和输出隔离。后续必须让管理器真正读取
`runtime.json` 的 `instances[]`，包括：

- `instance_id`
- `scenario`
- `flight_control`
- `faults`
- `random_seed`
- `enabled`

关于 GPU 或 SIMD 并行化，当前代码没有实现 GPU 后端。设计上只保留如下边界：

- 单实例闭环语义不能因并行化改变。
- 实例之间仍不得共享可变状态。
- 可并行化对象优先选择批量运行中的独立实例、传感器批处理、气动表插值和统计后处理。
- GPU 加速属于 P8 之后的性能扩展，不作为当前 P5-P7 正确性验收条件。

### 21.6 后续实现的完成判据

P5 完成判据：

- `faults.json` 被环境程序读取和校验。当前基础能力已实现。
- 故障按仿真时间触发，能作用于传感器有效位、测量值或执行机构。当前基础能力已实现。
- 触发、持续、恢复和拒绝原因写入 `event_log.txt`。当前开始/恢复事件已实现。
- `summary.json` 和成功实例的 `campaign_summary.json` 汇总故障统计。当前基础能力已实现。
- 闭环测试覆盖延迟预热、丢包、至少一种脚本故障和固定种子双跑一致性。当前基础能力已实现。

P6 完成判据：

- 飞控模块形成可测试静态库。
- PNG 有独立单元测试，覆盖 LOS 方向、限幅和异常输入。
- 加速度变化率限制接入主链路。
- 状态机、命令保持、传感器超时和 NaN/Inf 保护有闭环回归。

P7 完成判据：

- 管理器读取逐实例配置，而不是硬编码 baseline 路径。
- 支持并验证 `PARALLEL`、`SEQUENTIAL`、并发上限和失败继续策略。
- 单实例失败不会终止其他实例。
- `campaign_summary.json` 汇总每个实例的退出原因、最小距离和故障统计。

P8 完成判据：

- `sensor_log.bin` 可回放驱动飞控。
- `command_log.bin` 可转换为可读格式。
- 批量运行能输出脱靶量统计、成功率、失败原因分布和配置快照。
- 回放结果在固定种子下可重复。
