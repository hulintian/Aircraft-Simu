# 工具使用指南

> 更新时间：2026-08-20
> 本文说明 `tools/` 下各命令行工具的用途、输入输出和常见组合方式。

## 1. 使用前准备

先完成构建：

```sh
CCACHE_DISABLE=1 cmake -S . -B build
CCACHE_DISABLE=1 cmake --build build
```

所有工具默认位于：

```text
build/tools/<tool_name>/<tool_name>
```

大多数工具处理仿真产物。先跑一次多实例基线最方便：

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json
```

基线输出通常位于：

```text
runs/baseline_dev_001/
  campaign_summary.json
  instance_0000/
    sensor_log.bin
    command_log.bin
    summary.json
    trajectory.csv
    trajectory_diagnostics.csv
```

## 2. 工具总览

| 工具 | 作用 | 常用输入 | 常用输出 |
|---|---|---|---|
| `instance_manager` | 启动一个或多个飞控-环境闭环实例 | `runtime.json` | `campaign_summary.json`、各实例运行目录 |
| `log_convert` | 把协议或飞控内部二进制日志转为 CSV | sensor、command 或 `fc_internal_log.bin` | CSV 或 stdout |
| `replay` | 用传感器日志离线重新驱动飞控 | `sensor_log.bin`、`flight_control.json` | 新的 `command_log.bin` |
| `compare_logs` | 比较两份协议日志或真值轨迹 | 两份 sensor/command 日志或 `trajectory.csv` | JSON 对比报告或 stdout |
| `batch_stats` | 聚合单实例或批次摘要 | `summary.json`、`campaign_summary.json` | `stats.json` 或 stdout |
| `map_preprocess` | 把高程网格转成内部地形瓦片 | 裸 ASCII、ESRI ASCII Grid、SRTM HGT | `*.tile`、可选二进制索引 |
| `batch_runner` | 顺序运行多个 runtime，或生成 Monte Carlo runtime 清单 | runtime 清单或模板 | 多个批次输出、可选聚合统计 |
| `control_quality_report` | 执行标准飞控机动并按阈值验收 | 飞控配置、criteria JSON | 控制品质 JSON 报告 |
| `tools/plot/*.py` | 生成轨迹、状态、诊断和批次统计图 | `trajectory.csv`、`trajectory_diagnostics.csv`、`campaign_summary.json` | PNG |

## 3. instance_manager

用途：按 `runtime.json` 启动飞控和环境进程，管理端口、输出目录、实例种子和失败策略。

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json
```

参数：

- `--runtime PATH`：运行时配置。省略时使用程序默认路径。
- `--help`：打印参数帮助并成功退出。
- `--version`：打印项目和协议版本并成功退出。

重点看这些输出：

- `runs/<campaign>/campaign_summary.json`
- `runs/<campaign>/instance_XXXX/run_manifest.json`
- `runs/<campaign>/instance_XXXX/summary.json`
- `runs/<campaign>/instance_XXXX/sensor_log.bin`
- `runs/<campaign>/instance_XXXX/command_log.bin`

`run_manifest.json` 包含 Git/构建身份、逐配置 CRC32 和实际输入快照路径；
`campaign_summary.json` 包含互斥的命中/未命中/超时计数、脱靶量 min/max/mean/std
和失败实例列表。

`runtime.json` 中的 `instances[]` 决定每个实例的 `instance_id`、配置路径、随机种子和启用状态。
实例之间不共享运行时状态。

## 4. log_convert

用途：把固定小端二进制协议日志转换为 CSV，便于用表格工具或 Python 检查。

转换控制命令日志：

```sh
./build/tools/log_convert/log_convert \
  --type command \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_log.csv
```

转换传感器日志：

```sh
./build/tools/log_convert/log_convert \
  --type sensor \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output runs/baseline_dev_001/instance_0000/sensor_log.csv
```

转换飞控内部日志：

```sh
./build/tools/log_convert/log_convert \
  --type fc-internal \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/fc_internal_log.bin \
  --output runs/baseline_dev_001/instance_0000/fc_internal_log.csv
```

参数：

- `--type sensor|command|fc-internal`：日志类型，必须和输入文件匹配。
- `--instance-id N`：只接受该实例号的报文。
- `--input PATH`：输入二进制日志。
- `--output PATH`：输出 CSV；省略时写到 stdout。
- `--manifest PATH`：可选，指定 `REPLAY_PASSIVE` 运行清单。省略时使用
  `<output>.run_manifest.json`；若输出到 stdout，则使用
  `<input>.replay_passive.run_manifest.json`。

清单记录输入/输出路径、流式 CRC32、字节数、记录数、协议和构建身份。被动回放不重新
计算随机过程，因此 `random_seed` 为 `null`，并明确标记种子只存在于历史运行 provenance。

常见用途：

- 快速看飞控命令是否被限幅或保持。
- 检查导引头延迟预热期是否产生无效测量。
- 把 `command_log.csv` 和 `trajectory.csv` 放在一起做离线分析。

## 5. replay

用途：读取 `sensor_log.bin`，离线调用飞控静态库，重新生成控制命令日志。
它用于确认“同一飞控配置 + 同一传感器输入”能稳定复现命令输出。

```sh
./build/tools/replay/replay \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output runs/baseline_dev_001/instance_0000/replayed_command_log.bin
```

参数：

- `--instance-id N`：日志中的实例号。
- `--config PATH`：飞控配置。
- `--input sensor_log.bin`：原始传感器日志。
- `--output command_log.bin`：回放生成的控制命令日志。
- `--manifest PATH`：可选，指定 `REPLAY_WITH_FC` 运行清单；省略时使用
  `<output>.run_manifest.json`。

回放清单记录飞控配置 schema/CRC、传感器输入 CRC、命令输出 CRC、输入帧数、输出命令数、
Git/编译身份和协议版本。回放失败时工具返回非零；成功清单中的 `status` 为 `SIM_OK`。

常见用途：

- 修改飞控前后做命令一致性检查。
- 排查闭环结果变化是飞控变化还是环境变化。
- 给 `compare_logs` 准备右侧输入。

## 6. compare_logs

用途：比较两份二进制协议日志或两份 `trajectory.csv`，定位首个发散帧和数值差异。

比较原始命令日志和回放命令日志：

```sh
./build/tools/compare_logs/compare_logs \
  --type command \
  --instance-id 0 \
  --left runs/baseline_dev_001/instance_0000/command_log.bin \
  --right runs/baseline_dev_001/instance_0000/replayed_command_log.bin \
  --tolerance-config configs/verification/log_compare_exact.json \
  --output runs/baseline_dev_001/instance_0000/command_compare.json
```

参数：

- `--type command|sensor|trajectory`：比较控制命令、传感器日志或真值轨迹 CSV。
- `--instance-id N`：只接受该实例号的报文。
- `--left PATH`：基准协议日志或轨迹 CSV。
- `--right PATH`：待比较协议日志或轨迹 CSV。
- `--abs-tol X`：绝对容差；省略时使用工具默认值。
- `--rel-tol X`：相对容差；省略时使用工具默认值。
- `--tolerance-config FILE`：版本化 JSON 容差；命令行绝对/相对容差会覆盖文件值。
- `--output result.json`：输出 JSON；省略时写到 stdout。

比较模式：

- `log_compare_exact.json`：绝对/相对容差均为零。协议日志按完整字节比较，轨迹按完整表头和数据行比较；报告为 `EXACT`。
- `log_compare_tolerance.json`：按浮点绝对/相对阈值比较；协议字段、状态位、实例号、序号和模式仍精确匹配；报告为 `TOLERANCE`。

常见用途：

- 回放一致性检查：`command_log.bin` 对比 `replayed_command_log.bin`。
- 固定种子回归：两次运行的 `sensor_log.bin` 或 `command_log.bin` 对比。
- 环境真值回归：两次运行的 `trajectory.csv` 精确或容差对比。
- 协议或飞控改动后定位第一个行为差异。

## 7. batch_stats

用途：聚合单实例 `summary.json` 或批次 `campaign_summary.json`，生成更适合批量比较的统计文件。

聚合一个批次：

```sh
./build/tools/batch_stats/batch_stats \
  --input runs/baseline_dev_001/campaign_summary.json \
  --output runs/baseline_dev_001/batch_stats.json
```

聚合多个输入：

```sh
./build/tools/batch_stats/batch_stats \
  --input runs/case_a/campaign_summary.json \
  --input runs/case_b/campaign_summary.json \
  --output runs/batch_compare_stats.json
```

参数：

- `--input summary.json`：可重复指定；既可传单实例摘要，也可传批次摘要。
- `--output stats.json`：输出统计 JSON；省略时写到 stdout。

统计内容包括命中率、脱靶量样本数/min/max/mean/std、失败实例和失败原因分布、
故障影响步数、诊断采样数、四元数误差、DCM 正交误差、质量/惯量下限、
模型降级 flags、气动 flags、气动外推采样数和 wall-clock 耗时。

## 8. map_preprocess

用途：把外部高程网格转成项目内部地形瓦片格式，供 `scenario.json` 的
`map.tile_path`、`map.tile_paths[]` 或 `map.tile_index_path` 加载。

裸 ASCII 网格需要显式给出尺寸和经纬度范围：

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

ESRI ASCII Grid 会读取头部元数据：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input dem.asc \
  --input-format esri-ascii \
  --output terrain.tile \
  --nodata-fill 0 \
  --index-output terrain_index.bin
```

SRTM HGT 会从 `N30E120.hgt` 这类文件名推导 1 度瓦片范围：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input N30E120.hgt \
  --input-format srtm-hgt \
  --output terrain.tile \
  --nodata-fill 0 \
  --index-output terrain_index.bin
```

参数：

- `--input PATH`：输入高程文件。
- `--input-format esri-ascii|srtm-hgt`：输入格式；省略时按裸 ASCII 网格处理。
- `--output PATH`：输出内部瓦片。
- `--width W`、`--height H`：裸 ASCII 网格尺寸。
- `--lat-min-deg A`、`--lat-max-deg B`、`--lon-min-deg C`、`--lon-max-deg D`：裸 ASCII 网格范围。
- `--height-scale S`：高程缩放，默认按工具内部默认值处理。
- `--height-offset O`：高程偏置，默认按工具内部默认值处理。
- `--nodata-fill H`：ESRI ASCII Grid 或 HGT 缺测值填补高度。
- `--index-output PATH`：可选，写出二进制空间索引。

注意：

- SRTM HGT 的 void 值是 `-32768`，实际使用时应显式给 `--nodata-fill`。
- 索引中的相对瓦片路径按索引文件所在目录解析，便于移动整个 DEM 目录。
- `tests/fixtures/dem/` 提供带来源、SHA-256 和回归预期的珠峰附近 9x9 真实来源最小 fixture。
- 生产 DEM 的许可证、完整性、覆盖范围和高程基准仍需在独立资源清单中记录。

## 9. batch_runner

用途有两个：

1. 按清单顺序运行多个 `runtime.json`。
2. 从 runtime 模板生成确定性 Monte Carlo runtime 清单。

### 9.1 运行已有清单

清单文件每行包含一个 runtime 路径；可选第二列写该 runtime 对应的摘要路径，供后续聚合使用。

```text
configs/baseline/runtime.json runs/baseline_dev_001/campaign_summary.json
```

运行清单：

```sh
./build/tools/batch_runner/batch_runner \
  --manifest batch_runs.txt \
  --instance-manager ./build/tools/instance_manager/instance_manager \
  --batch-stats ./build/tools/batch_stats/batch_stats \
  --output batch_stats.json
```

参数：

- `--manifest runs.txt`：runtime 清单。
- `--instance-manager PATH`：要调用的 `instance_manager`。
- `--batch-stats PATH`：可选，运行结束后调用 `batch_stats`。
- `--output stats.json`：可选，聚合统计输出路径。
- `--stop-on-failure`：任一 runtime 失败后停止后续运行。
- `--run-manifest PATH`：可选，指定 `MONTE_CARLO` 工作流清单；省略时使用
  `<runs.txt>.run_manifest.json`。

### 9.2 生成 Monte Carlo runtime 清单

```sh
./build/tools/batch_runner/batch_runner \
  --generate-manifest batch_runs.txt \
  --runtime-template runtime_template.json \
  --runtime-output-dir runs/mc_batch \
  --sample-count 32 \
  --base-seed 10000
```

参数：

- `--generate-manifest runs.txt`：生成的清单路径。
- `--runtime-template template.json`：runtime 模板。
- `--runtime-output-dir DIR`：生成 runtime 和样本输出目录的位置。
- `--sample-count N`：样本数。
- `--base-seed SEED`：基础种子；省略时使用工具默认值。
- `--instance-manager PATH`：可选；生成后也可以继续调用管理器运行。
- `--run-manifest PATH`：可选，覆盖默认 `<runs.txt>.run_manifest.json`。

模板中可使用这些占位符：

- `${sample_index}`
- `${random_seed}`
- `${sample_output_dir}`
- `${uniform:stream:min:max}`
- `${lhs_uniform:stream:min:max}`
- `${halton_uniform:base:min:max}`
- `${normal:stream:mean:stddev}`
- `${lognormal:stream:mu:sigma}`
- `${truncated_normal:stream:mean:stddev:min:max}`
- `${choice:stream:option|option}`
- `${correlated_normal:stream:base_stream:mean:stddev:rho}`

同一 `base_seed`、样本序号和 stream 会生成确定性扰动，适合可重复批量回归。
工作流清单记录模板和运行列表 CRC、基础种子策略、请求/实际样本数、尝试/完成/失败数量、
聚合统计路径和构建身份。各样本的具体配置版本与种子仍以生成的 runtime 和子运行
`run_manifest.json` 为准。

## 10. 绘图工具

`tools/plot/` 下提供一组报告级 PNG 绘图工具。它们不参与 CMake 构建，
直接用 `python3` 运行。依赖：

```sh
python3 -c "import pandas, numpy, matplotlib"
```

### 10.1 单实例一键出图

```sh
python3 tools/plot/plot_run.py \
  --instance-dir runs/baseline_dev_001/instance_0000 \
  --output-dir runs/baseline_dev_001/instance_0000/plots \
  --title-prefix baseline
```

输出：

- `trajectory.png`：ECEF 三维轨迹、经纬度地面航迹、高度和距离曲线。
- `timeseries.png`：距离、闭合速度、速度范数、加速度范数、高度/AGL、质量/合力。
- `diagnostics.png`：四元数误差、DCM 正交误差、质量/惯量、力矩和模型 flags。

### 10.2 单独绘制轨迹图

```sh
python3 tools/plot/plot_trajectory.py \
  --trajectory runs/baseline_dev_001/instance_0000/trajectory.csv \
  --output runs/baseline_dev_001/instance_0000/trajectory.png \
  --title "baseline instance 0 trajectory"
```

### 10.3 单独绘制状态时序图

```sh
python3 tools/plot/plot_timeseries.py \
  --trajectory runs/baseline_dev_001/instance_0000/trajectory.csv \
  --output runs/baseline_dev_001/instance_0000/timeseries.png
```

### 10.4 单独绘制数值诊断图

```sh
python3 tools/plot/plot_diagnostics.py \
  --diagnostics runs/baseline_dev_001/instance_0000/trajectory_diagnostics.csv \
  --output runs/baseline_dev_001/instance_0000/diagnostics.png
```

### 10.5 绘制批次统计图

```sh
python3 tools/plot/plot_campaign.py \
  --campaign runs/baseline_dev_001/campaign_summary.json \
  --output runs/baseline_dev_001/campaign_summary.png \
  --title "baseline campaign"
```

该图包含实例脱靶量散点、脱靶量分布、实例 wall-clock 耗时和完成/失败/命中计数。

## 11. control_quality_report

用途：使用实际 `FlightController` 链执行阶跃、指令反向和丢包恢复标准机动，计算
上升时间、调节时间、超调、稳态误差、最大变化率和饱和占比。

```sh
./build/tools/control_quality/control_quality_report \
  --flight-control configs/baseline/flight_control.json \
  --criteria configs/verification/control_quality.json \
  --output control_quality_report.json
```

参数：

- `--flight-control PATH`：被验收的飞控配置。
- `--criteria PATH`：版本化机动与阈值配置。
- `--output PATH`：JSON 报告；省略时写到 stdout。

报告的证据范围是工程 SIL 控制链，不代表真实对象稳定裕度或型号飞行品质。

短中长压力测试、真实 DEM 证据和发布前检查见
[验证与验收指南](verification_guide.md)。

## 12. 常见工作流

### 12.1 跑一次仿真并查看命令

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json

./build/tools/log_convert/log_convert \
  --type command \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_log.csv
```

### 12.2 做飞控回放一致性检查

```sh
./build/tools/replay/replay \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output runs/baseline_dev_001/instance_0000/replayed_command_log.bin

./build/tools/compare_logs/compare_logs \
  --type command \
  --instance-id 0 \
  --left runs/baseline_dev_001/instance_0000/command_log.bin \
  --right runs/baseline_dev_001/instance_0000/replayed_command_log.bin \
  --tolerance-config configs/verification/log_compare_exact.json \
  --output runs/baseline_dev_001/instance_0000/command_compare.json
```

### 12.3 做一组批量运行并聚合

```sh
./build/tools/batch_runner/batch_runner \
  --manifest batch_runs.txt \
  --instance-manager ./build/tools/instance_manager/instance_manager \
  --batch-stats ./build/tools/batch_stats/batch_stats \
  --output batch_stats.json
```

## 13. 排错

- 工具提示 `bad packet` 或 CRC 错误：确认 `--type` 是否和日志文件匹配，确认日志没有截断。
- 工具提示 instance 不匹配：确认 `--instance-id` 和运行实例一致。
- `instance_manager` 启动失败：检查 UDP 端口是否被占用，检查 `runtime.tools` 中的子程序路径。
- `replay` 输出和原始命令不同：先确认飞控配置相同，再用 `compare_logs` 查首个差异帧。
- `map_preprocess` 拒绝输入：检查网格尺寸、经纬度范围、HGT 文件名格式和缺测值填补。
- 批量统计缺字段：确认输入是本项目生成的 `summary.json` 或 `campaign_summary.json`。
- 绘图工具导入失败：确认当前 Python 环境可导入 `pandas`、`numpy` 和 `matplotlib`。
