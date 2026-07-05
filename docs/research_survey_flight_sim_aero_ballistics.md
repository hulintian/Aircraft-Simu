# 飞行器飞行仿真、空气动力学与火箭弹道研究调研

日期：2026-07-01  
范围：公开论文、开源仿真工具和本项目当前实现。  
项目基线：本文保留为阶段性研究背景。当前实现进度、模块状态和可运行命令以
`README.md`、`docs/project_framework.md` 和 `docs/onboarding.md` 为准。
与成熟飞行仿真软件的横向差距见
`docs/flight_sim_software_comparison.md`。

## 1. 摘要

近两年的飞行器仿真研究有三个明显方向：

1. **6DOF 刚体动力学仍是工程主干**。新的工作更强调数学形式的一致性、姿态四元数约束、直接/逆动力学统一，以及跨仿真器验证。
2. **气动模型正在从固定系数表转向“低阶工程模型 + 高保真 CFD/风洞数据 + 代理模型”的组合**。神经场、神经算子、物理约束机器学习和贝叶斯优化被用于快速扫掠飞行包线、几何参数和控制面偏转。
3. **弹道/火箭轨迹优化开始重视离散化本身的失效模式**。2026 年关于 6DOF 火箭着陆轨迹优化的研究显示，积分/转录格式会引入截断误差和四元数不变量漂移，普通收敛状态并不等价于连续动力学可行。

对本项目的直接含义是：当前 `environment_sim` 已经有 ECEF 真值、6DOF、RK4、重力/大气/气动/推进/执行机构链路，方向是正确的；下一步不应盲目堆复杂模型，而应优先补齐 **气动数据接口、回放验证、数值不变量监测、批量不确定性分析和真实地形/大气资源链路**。

## 2. 本项目当前状态与研究问题

本项目当前已经具备：

- 双进程 UDP LOCKSTEP：`environment_sim -> SensorFrame -> flight_control_sim -> ControlCommand -> environment_sim`。
- ECEF 主真值坐标、WGS-84、LLA/ECEF/ENU/NED、AGL、地表碰撞和 LOS 遮挡接口。
- 6DOF 刚体状态、RK4 积分、重力、ISA 大气、低阻力气动、推进、质量消耗和地球自转项。
- 四类传感器误差、延迟、丢包和基础故障脚本。
- 飞控 PNG、状态机、估计、自动驾驶仪、舵面分配、命令限幅/变化率限制。
- 多实例管理器、二进制日志和 `log_convert`。

当前主要限制：

- 气动仍是低阶参数模型，缺少 Mach/AoA/舵偏/高度相关的气动数据库。
- 惯量只按质量比例近似变化，未建模质心迁移和完整惯量张量演化。
- 地形瓦片格式和算法已实现，但真实 DEM 加载链和预处理工具未完成。
- P8 仍缺传感器日志回放驱动飞控、命令一致性比较、批量统计和回归阈值。
- 飞控内部保护动作持久化日志仍未完成。

调研的核心问题是：如何把当前教育/工程架构型闭环仿真，推进为可验证、可扩展、可批量回归的飞行器仿真平台。

## 3. 研究趋势

### 3.1 6DOF 飞行动力学：从可运行到可验证

Marzouk 2025 提出面向非对称固定翼飞机的 6DOF 微分-代数方程框架，强调风轴/体轴变量组织、直接/逆动力学统一、四元数避免欧拉角奇异，并考虑随高度变化的空气密度。这类研究对本项目的启发不是照搬固定翼模型，而是把 6DOF 状态、输入、约束和输出整理成可验证的数学接口。

JSBSim 的工程路线也值得参考。它把飞行器质量、几何、推进、控制系统、自动驾驶仪和气动稳定导数放入 XML/属性树，而不是把具体机型写死在代码里；同时支持批处理、脚本测试和可配置日志。这个思想与本项目 JSON 配置、二进制协议和多实例日志方向一致。

对本项目建议：

- 在 `missile_plant_6dof` 中增加数值健康量：四元数范数误差、DCM 正交误差、能量/动量类诊断项、质量/惯量正定性。
- 把气动力/力矩接口从当前低阶系数扩展为 `AeroDatabase`：输入 Mach、AoA、侧滑角、舵偏、高度或雷诺数，输出 `force_b`、`moment_b` 或系数。
- 增加验证算例：真空抛体、恒推力、恒力矩转动、无气动弹道、纯阻力弹道、固定姿态气动扫掠。

### 3.2 气动建模：高保真 CFD 与快速代理模型并行

AirfRANS 数据集展示了空气动力学机器学习基准化趋势：用高保真 RANS 数据训练代理模型，并把评估指标从单纯场误差扩展到升阻力、壁面应力、边界层和外推能力。NeurIPS 2024 ML4CFD 进一步强调工业采用前必须评估精度、计算效率、分布外表现和物理一致性。

2025 年 NeuralFoil 把物理启发特征、解析约束和机器学习结合，用于快速翼型气动分析。其意义不在于本项目需要引入 Python 神经网络，而在于它说明“代理模型可用”的前提是：输入空间明确、物理边界条件明确、外推行为受控，并且要有不确定性估计。

2025 年 MARIO 神经场代理模型进一步把问题推进到大规模气动仿真：通过形状编码和分辨率不变预测，在 AirfRANS 与 NASA Common Research Model 上验证速度场、压力场、湍流粘性和气动系数预测。2024 年 DeepONet 用于 Ma=7.36 高超声速 waverider 的压力、密度、速度、热流和剪切应力预测，说明神经算子正在进入带激波和强不连续的气动热问题。

对本项目建议：

- 短期不要直接把神经网络塞入 C 主循环。先定义稳定的气动表接口和离线数据格式。
- 气动模型分三档：
  - `AERO_SIMPLE`：当前低阻力/控制力矩模型，用于闭环和测试。
  - `AERO_TABLE`：Mach/AoA/舵偏表插值，作为近期主目标。
  - `AERO_SURROGATE`：离线训练、在线只读推理，后续再接。
- 每个气动模型必须输出模型来源、适用包线、插值状态、外推标志和不确定度占位。
- 测试重点从“数值相等”扩展到“包线内连续、边界有限、外推可报警、力/矩方向合理”。

### 3.3 火箭/弹道轨迹：积分器、离散化和不确定性是核心风险

2026 年 Sharma 等关于 6DOF 火箭着陆轨迹优化的研究非常值得本项目吸收。该文指出，连续动力学被转录为离散 NLP 后，局部截断误差和四元数不变量漂移会导致动态不可行或次优轨迹；在 14 种转录方法中，只有少数通过严格验证。对本项目而言，即使我们不是做着陆最优控制，也同样存在“RK4 跑通不等于所有飞行包线可靠”的问题。

Nurre 与 Taheri 2025 的 6DOF powered descent guidance 工作则强调，6DOF 动力学下的姿态约束、推力约束、角速度约束、滑翔斜率约束会让问题高度非线性。Dymos/OpenMDAO 的路线表明，现代轨迹优化工具强调相位、转录、解析导数、稀疏性和多学科耦合，而不是只写一个积分循环。

2025 年关于模型火箭气动参数的 amortized inference 工作展示了另一路线：用物理仿真产生合成数据，训练网络从飞行结果反推阻力系数和推力修正，再迁移到真实飞行。它对本项目的启发是：即使没有真实型号数据，也可以用本仿真器做参数识别、误差归因和 sim-to-real gap 分析。

对本项目建议：

- P8 回放工具应优先实现，因为它是数值回归和控制律回归的基石。
- 增加积分器对照测试：Euler/RK2/RK4 在同一场景下比较误差、四元数漂移和命中统计。
- 对 `dt`、气动系数、传感器延迟、丢包率、风场、质量、推力建立 Monte Carlo 批量扫描。
- 输出 `trajectory_diagnostics.csv`：四元数范数、DCM 正交误差、总质量、惯量最小特征值、气动包线标志、最大指令变化率等。

### 3.4 仿真验证：单次命中不够，必须可回放、可分散、可审计

JSBSim、Dymos 和近年 ML4CFD 共同体现一个趋势：仿真平台的价值不只在模型本身，还在可配置、可批处理、可复现、可验证和可比较。当前项目已经有二进制日志、`summary.json`、`campaign_summary.json` 和多实例管理器，但 P8 尚未完成。

对本项目建议：

- `tools/replay`：读取 `sensor_log.bin`，重新驱动 `flight_control_sim` 或飞控静态库，输出新的 `command_log.bin`。
- `tools/compare_logs`：逐帧比较 ControlCommand，支持绝对/相对阈值、状态位差异和首个发散帧定位。
- `tools/batch_stats`：汇总 `summary.json` 和 `campaign_summary.json`，输出命中率、最小距离均值/标准差、失败原因分布、故障影响统计。
- CI 中拆分测试层：纯单元测试、UDP 闭环测试、长时间/Monte Carlo 可选测试。

## 4. 重点文献与工具表

| 方向 | 代表来源 | 关键结论 | 对本项目的用法 |
|---|---|---|---|
| 6DOF 飞行动力学 | Marzouk, 2025, 6DOF DAE fixed-wing framework | 直接/逆动力学统一，四元数规避奇异，非对称惯量可处理 | 整理 6DOF 接口、补惯量和姿态验证 |
| 开源 FDM 工程 | JSBSim 1.3.1 文档 | 数据驱动配置、批处理、脚本、可配置日志 | 借鉴属性/配置驱动和日志设计 |
| 最优控制工具 | Dymos/OpenMDAO 文档 | 相位、转录、解析导数和多学科耦合 | P8 后可做离线轨迹优化/参数扫描 |
| 气动 ML 基准 | AirfRANS, 2022；ML4CFD, 2024 | 代理模型需统一数据集、OOD 和物理一致性评估 | 建立气动表/代理模型测试标准 |
| 快速翼型气动 | NeuralFoil, 2025 | 物理启发 ML 可快速预测翼型气动并带 UQ | 后续 `AERO_SURROGATE` 的设计参考 |
| 大规模气动代理 | MARIO, 2025 | 神经场可处理分辨率不变和复杂 3D 气动预测 | 离线高保真数据到在线模型的长期路线 |
| 高超声速气动热 | Shukla et al., 2024, DeepONet waverider | 神经算子用于激波/热流/剪切应力代理 | 说明高 Mach 场景需单独包线和验证 |
| 近空间多尺度 | Xi et al., 2026, IUGKS + surrogate optimization | 稀薄流/连续流跨域时气动主导因素会变 | 不要让 ISA/连续流模型越界无报警 |
| 火箭 6DOF 转录失效 | Sharma et al., 2026 | 离散化可导致动态不可行和四元数漂移 | 增加积分器/不变量/回放一致性测试 |
| 参数反演 | Pandey & Pandey, 2025, model rocket amortized inference | 合成仿真数据可辅助反推气动参数 | 做阻力/推力修正参数识别实验 |

## 5. 与本项目的推荐路线

### 5.1 近期：补齐 P8，建立验证闭环

优先级最高，不依赖复杂模型。

1. `tools/replay`
   - 输入：`sensor_log.bin`、`flight_control.json`、`instance_id`。
   - 输出：`replayed_command_log.bin`、`replay_summary.json`。
   - 验收：同一飞控版本下逐帧命令一致。

2. `tools/compare_logs`
   - 比较原始 `command_log.bin` 与回放命令。
   - 支持浮点阈值、状态位差异、首个发散帧输出。

3. `tools/batch_stats`
   - 汇总多实例最小距离、命中率、故障次数、失败原因。
   - 输出 CSV/JSON，支持回归阈值。

### 5.2 中期：升级气动模型，不破坏主链路

1. 增加 `aero_database.h/c`。
2. JSON 增加：

```json
{
  "aerodynamics": {
    "model": "TABLE",
    "database_path": "data/aero/example_table.csv",
    "inputs": ["mach", "alpha_rad", "beta_rad", "pitch_fin_rad", "yaw_fin_rad"],
    "outputs": ["cx", "cy", "cz", "cl", "cm", "cn"],
    "extrapolation_policy": "CLAMP_AND_WARN"
  }
}
```

3. 主循环仍只消费 `force_b` 和 `moment_b`，避免飞控或协议被气动数据格式污染。
4. 单元测试覆盖插值、边界、缺表、NaN、外推报警和力矩方向。

### 5.3 中期：数值稳定与物理不变量测试

增加：

- `plant_diagnostics`：四元数范数、DCM 正交性、惯量正定性、质量正性。
- `integrator_regression_test`：恒力、恒力矩、无气动、纯阻力、推进剂耗尽。
- `dt_sensitivity_test`：`dt = 0.02/0.01/0.005` 下命中统计和轨迹误差趋势。

### 5.4 长期：代理模型与参数识别

在 C 主链路稳定后再做：

- 离线 Python 生成气动代理模型，不直接改变 C 主循环。
- 训练输出应转成只读表、ONNX 或项目自定义静态权重格式。
- 先做阻力系数/推力修正的低维参数识别，再考虑高维气动场代理。
- 所有代理模型必须带适用包线和不确定度/置信标志。

## 6. 风险与边界

1. **研究文献的先进性不等于工程可接入性**。神经算子和神经场有价值，但本项目当前最缺的是接口、验证和数据治理。
2. **不要让模型外推静默发生**。气动表、地形、传感器和大气模型都应有包线状态位。
3. **不要把“单次闭环通过”当成可信仿真**。必须有回放、批量统计、阈值回归和故障注入。
4. **双用途边界**。本项目应保持软件仿真、教学验证和工程架构研究定位，不引入真实装备参数、实装控制接口或可直接迁移到真实系统的操作流程。

## 7. 建议的实现顺序

1. P8 `replay + compare_logs + batch_stats`。
2. 飞控内部保护动作持久化日志。
3. `plant_diagnostics` 和积分器/不变量回归测试。
4. `aero_database` 表格模型和包线报警。
5. 真实 DEM 瓦片加载链和 `map_preprocess`。
6. Monte Carlo 批量运行配置与统计阈值。
7. 低维参数识别实验：阻力系数、推力修正、传感器偏置。
8. 气动代理模型离线原型。

## 8. 参考资料

1. Marzouk, O. A. (2025). Coupled differential-algebraic equations framework for modeling six-degree-of-freedom flight dynamics of asymmetric fixed-wing aircraft. [arXiv:2412.17280](https://arxiv.org/abs/2412.17280).
2. JSBSim Team. JSBSim Flight Dynamics Model 1.3.1 documentation. [https://jsbsim-team.github.io/jsbsim/](https://jsbsim-team.github.io/jsbsim/).
3. Dymos Development Team. Multidisciplinary Optimal Control Library documentation. [https://openmdao.org/dymos/docs/latest/index.html](https://openmdao.org/dymos/docs/latest/index.html).
4. Bonnet, F., Mazari, A. J., Cinnella, P., & Gallinari, P. (2022). AirfRANS: High Fidelity Computational Fluid Dynamics Dataset for Approximating Reynolds-Averaged Navier-Stokes Solutions. [arXiv:2212.07564](https://arxiv.org/abs/2212.07564).
5. Yagoubi, M., et al. (2024). NeurIPS 2024 ML4CFD Competition: Harnessing Machine Learning for Computational Fluid Dynamics in Airfoil Design. [arXiv:2407.01641](https://arxiv.org/abs/2407.01641).
6. Sharpe, P., & Hansman, R. J. (2025). NeuralFoil: An Airfoil Aerodynamics Analysis Tool Using Physics-Informed Machine Learning. [arXiv:2503.16323](https://arxiv.org/abs/2503.16323).
7. Catalani, G., et al. (2025). Towards scalable surrogate models based on Neural Fields for large scale aerodynamic simulations. [arXiv:2505.14704](https://arxiv.org/abs/2505.14704).
8. Shukla, K., et al. (2024). Deep operator learning-based surrogate models for aerothermodynamic analysis of AEDC hypersonic waverider. [arXiv:2405.13234](https://arxiv.org/abs/2405.13234).
9. Xi, X., Long, W., Guo, W., Cao, J., & Xu, K. (2026). Surrogate-Based Aerodynamic Shape Optimization in Multiscale Flows via the Implicit Unified Gas-Kinetic Scheme. [arXiv:2606.00645](https://arxiv.org/abs/2606.00645).
10. Sharma, P., Goh, J. Y. M., Açıkmeşe, B., & Djeumou, F. (2026). Transcription-Induced Failure Modes in 6-DOF Rocket Landing Trajectory Optimization. [arXiv:2605.08420](https://arxiv.org/abs/2605.08420).
11. Nurre, N. P., & Taheri, E. (2025). Constrained Fuel and Time Optimal 6DOF Powered Descent Guidance Using Indirect Optimization. [arXiv:2501.14173](https://arxiv.org/abs/2501.14173).
12. Pandey, R., & Pandey, R. (2025). Amortized Inference for Model Rocket Aerodynamics: Learning to Estimate Physical Parameters from Simulation. [arXiv:2512.22248](https://arxiv.org/abs/2512.22248).
13. Jarry, G., Dalmau, R., Olive, X., & Very, P. (2025). A Neural ODE Approach to Aircraft Flight Dynamics Modelling. [arXiv:2509.23307](https://arxiv.org/abs/2509.23307).
14. Carlson, K., & Renganathan, A. (2025). Multiobjective Aerodynamic Design Optimization of the NASA Common Research Model. [arXiv:2507.10488](https://arxiv.org/abs/2507.10488).

## AI 使用披露

本文由 AI 辅助完成，包含公开文献检索、来源筛选、综合分析和面向本项目的工程化改写。文中技术建议限于软件仿真、教学验证和工程架构研究，不包含真实装备参数或实装操作流程。
