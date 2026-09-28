# 验证与验收指南

> 更新日期：2026-08-20
> 本文给出本仓库统一的构建、回归、压力、控制品质和证据边界。

## 1. 验证层级

| 层级 | 含义 | 当前状态 |
|---|---|---|
| V0 | 代码和配置可构建、严格警告通过 | 仓库自动验证 |
| V1 | 数学模块、协议和模型单元正确 | 仓库自动验证 |
| V2 | 解析算例、不变量、文件 CRC 和错误路径正确 | 仓库自动验证 |
| V3 | 双进程 SIL 因果闭环正确 | 仓库自动验证 |
| V4 | 固定种子可复现，日志可转换、回放和比较 | 仓库自动验证 |
| V5 | 多实例、故障、控制品质和短中长压力覆盖 | 仓库自动验证，长测需显式启用 |
| V6 | 真实气动/器件/地形数据标定和外部软件交叉验证 | 需要项目外数据，未完成 |
| V7 | HIL、台架、半实物或实测验证 | 需要外部硬件，未完成 |

P0-P8 的“完成”只表示 V0-V5 工程验收闭合，不表示真实型号精度达到 V6/V7。

## 2. 默认回归

```sh
CCACHE_DISABLE=1 cmake -S . -B build
CCACHE_DISABLE=1 cmake --build build
CCACHE_DISABLE=1 ctest --test-dir build --output-on-failure
```

默认测试包含公共库、环境、飞控、真实 DEM/v1/v2 气动表双进程闭环、多实例管理、
日志转换、回放比较、地图预处理、批跑、控制品质和 4 实例短压力测试。
UDP 测试需要允许本机 `127.0.0.1` 回环通信。

环境单测还覆盖脚本目标在步内机动边界的分段积分，以及相对轨迹穿过命中球但两个步端
均在球外的连续命中场景。闭环清单必须记录目标模型、机动段数量、积分器和模型资源。

## 3. 控制品质报告

```sh
./build/tools/control_quality/control_quality_report \
  --flight-control configs/baseline/flight_control.json \
  --criteria configs/verification/control_quality.json \
  --output control_quality_report.json
```

报告使用真实 `FlightController` 链执行加速度阶跃、指令反向和丢包恢复，输出上升时间、
调节时间、超调、稳态误差、最大变化率和饱和占比。阈值由版本化 criteria 文件管理。
该报告验证工程控制链和保护边界，不替代真实对象的带宽、稳定裕度或飞行品质试验。

## 4. 回放与容差基准

先回放传感器日志：

```sh
./build/tools/replay/replay \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output replayed_command_log.bin
```

固定平台确定性回放先使用版本化精确比较：

```sh
./build/tools/compare_logs/compare_logs \
  --type command \
  --instance-id 0 \
  --left runs/baseline_dev_001/instance_0000/command_log.bin \
  --right replayed_command_log.bin \
  --tolerance-config configs/verification/log_compare_exact.json \
  --output command_compare.json
```

`log_compare_exact.json` 的绝对/相对容差均为零，协议日志执行完整字节比较，报告记录
`"comparison_mode": "EXACT"`。跨平台浮点回归改用
`configs/verification/log_compare_tolerance.json`；命令行 `--abs-tol` 和 `--rel-tol`
可覆盖配置值。环境真值回归使用 `--type trajectory` 比较两份 `trajectory.csv`，
精确模式执行表头和数据行文本比较，容差模式执行逐数值比较。

## 5. 日志审计

```sh
./build/tools/log_convert/log_convert \
  --type fc-internal \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/fc_internal_log.bin \
  --output fc_internal_log.csv
```

`--type` 还支持 `sensor` 和 `command`。转换器按固定线格式解码并拒绝截断、CRC、
实例号或记录长度错误。`log_convert`、`replay` 和 `batch_runner` 分别写
`REPLAY_PASSIVE`、`REPLAY_WITH_FC` 和 `MONTE_CARLO` 工作流清单；验收时应同时保存
sidecar manifest，不只保存转换后的数据文件。

## 6. 地形与资源证据

真实来源最小 DEM fixture 位于 `tests/fixtures/dem/`：

- `everest_terrain_9x9.txt`：珠峰附近 9x9 高程裁剪。
- `map_manifest.json`：来源 URL、访问日期、源文件和派生文件 SHA-256、坐标范围和预期值。

`map_preprocess_test` 验证预处理、峰值、阻挡/放通 LOS；`closed_loop_test` 会让环境进程
加载该 fixture 派生瓦片并验证 LOS 遮挡保护。生产数据仍需独立记录许可证、完整性和适用范围。

## 7. 压力测试分档

默认 short：4 实例、并发 2，随默认 CTest 运行。

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_medium_build -DMISSILE_ENABLE_MEDIUM_TESTS=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_medium_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_medium_build \
  -R campaign_pressure_medium_test --output-on-failure
```

medium 为 16 实例、并发 4。long 为 128 实例、并发 8：

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_long_build -DMISSILE_ENABLE_LONG_TESTS=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_long_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_long_build \
  -R campaign_pressure_long_test --output-on-failure
```

三档均检查完成/失败数量、逐实例随机种子、诊断聚合和 wall-clock 字段。真实目标机器上的
长时资源占用、调度抖动和容量结论仍应另建基准记录。

## 8. 通信压力配置

`configs/verification/faults_communication_stress.json` 提供周期突发丢包、确定性变延迟和
周期上一帧重放示例。完整故障回归还覆盖传感器退化/饱和、执行机构卡滞/偏置/速率/
行程/延迟/失能，以及实际 UDP 报文丢弃、重复、CRC 损坏和乱序。确定性帧级压力保持
`LOCKSTEP` 因果关系；真实报文丢弃/损坏与恢复使用 `FREE_RUNNING`，它仍不是操作系统
网络栈、真实总线或 HIL 时序的替代品。

## 9. 覆盖率与 sanitizer

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_coverage_build -DMISSILE_ENABLE_COVERAGE=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_coverage_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_coverage_build --output-on-failure
```

```sh
CCACHE_DISABLE=1 cmake -S . -B /tmp/missile_sanitize_build -DMISSILE_ENABLE_SANITIZERS=ON
CCACHE_DISABLE=1 cmake --build /tmp/missile_sanitize_build
CCACHE_DISABLE=1 ctest --test-dir /tmp/missile_sanitize_build --output-on-failure
```

sanitizer 配置会先检查 ASan/UBSan 运行库；工具链缺少运行库时在配置阶段明确失败。

## 10. 发布前检查

1. 默认构建和全部默认 CTest 通过。
2. `git diff --check` 无空白错误。
3. 控制品质报告顶层 `pass` 为 `true`，三个 `maneuvers.*.pass` 均为 `true`。
4. 回放精确比较 verdict 为 `PASS` 且 `comparison_mode` 为 `EXACT`；跨平台基准另记录容差配置。
5. `run_manifest.json` 记录 Git/构建身份、配置 CRC32、输入快照、运行模式、标准日志路径和模型资源/包线。
6. 回放、转换和批跑产物具有成功状态的工作流 sidecar manifest，输入/输出 CRC 与实际文件一致。
7. 对外结论明确标注当前最高证据层级，不用 V0-V5 结果替代 V6/V7。
