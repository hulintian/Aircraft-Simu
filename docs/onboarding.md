# 新成员上手指南

> 更新时间：2026-07-04
> 目标读者：第一次打开本仓库、准备运行仿真、修改模型或排查测试失败的开发者。

## 1. 先读什么

按这个顺序读，避免被完整设计文档淹没：

1. `README.md`
   - 看项目定位、当前进度、构建命令和常用工具。
2. `docs/onboarding.md`
   - 跟着本文跑通构建、单实例、多实例、回放和日志转换。
3. `docs/project_framework.md`
   - 看实际源码结构、主数据流、当前实现状态和缺口。
4. `docs/flight_sim_software_comparison.md`
   - 理解本项目和成熟飞行仿真软件的边界差距。
5. `docs/design.md`
   - 改模型、协议、主循环或飞控链路前再深入读。
6. `docs/implementation_plan.md`
   - 继续实现计划项时读，确认阶段验收标准。

## 2. 一句话理解系统

一个实例由两个独立进程组成：

```text
environment_sim
  生成真值、传感器帧、地形/动力学、日志
        |
        | UDP SensorFrame
        v
flight_control_sim
  校验传感器、估计、制导、自动驾驶仪、保护、命令管理
        |
        | UDP ControlCommand
        v
environment_sim
  应用执行机构、推进 6DOF、写轨迹和摘要
```

环境进程拥有仿真时钟。当前默认是 LOCKSTEP：每个传感器帧都等待一条控制指令。

## 3. 构建和测试

首次构建：

```sh
CCACHE_DISABLE=1 cmake -S . -B build
CCACHE_DISABLE=1 cmake --build build
```

运行默认回归：

```sh
CCACHE_DISABLE=1 ctest --test-dir build --output-on-failure
```

注意：`closed_loop_test` 和 `instance_manager_test` 会创建本机 UDP socket。如果运行环境
限制网络命名空间，需要允许本地 `127.0.0.1` UDP。

## 4. 单实例运行

先启动飞控，再启动环境：

```sh
./build/flight_control_sim/flight_control_sim --instance-id 0 &
fc_pid=$!
sleep 0.2
./build/environment_sim/environment_sim --instance-id 0
wait "$fc_pid"
```

显式指定配置：

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

## 5. 多实例运行

```sh
./build/tools/instance_manager/instance_manager \
  --runtime configs/baseline/runtime.json
```

`runtime.json` 中的 `instances[]` 是逐实例计划。每个实例应有独立：

- `instance_id`
- UDP 端口
- 输出目录
- 随机种子
- 场景/飞控/故障配置路径

实例之间不通信，不共享运行时状态。

## 6. 输出在哪里

默认输出目录形如：

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

优先看：

- `summary.json`：本次运行是否命中、最近距离、退出原因、故障和诊断统计。
- `event_log.txt`：启动、故障开始/恢复、命中、停止事件。
- `trajectory.csv`：真值轨迹和距离变化。
- `trajectory_diagnostics.csv`：四元数、DCM、质量/惯量、气动 flags 等数值诊断。
- `run_manifest.json`：配置、端口、随机种子、地形/气动资源。

## 7. 常用工具

控制日志转 CSV：

```sh
./build/tools/log_convert/log_convert \
  --type command \
  --instance-id 0 \
  --input runs/baseline_dev_001/instance_0000/command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_log.csv
```

传感器日志回放飞控：

```sh
./build/tools/replay/replay \
  --instance-id 0 \
  --config configs/baseline/flight_control.json \
  --input runs/baseline_dev_001/instance_0000/sensor_log.bin \
  --output runs/baseline_dev_001/instance_0000/replayed_command_log.bin
```

比较两份日志：

```sh
./build/tools/compare_logs/compare_logs \
  --type command \
  --instance-id 0 \
  --left runs/baseline_dev_001/instance_0000/command_log.bin \
  --right runs/baseline_dev_001/instance_0000/replayed_command_log.bin \
  --output runs/baseline_dev_001/instance_0000/command_compare.json
```

批量统计：

```sh
./build/tools/batch_stats/batch_stats \
  --input runs/baseline_dev_001/campaign_summary.json \
  --output runs/baseline_dev_001/batch_stats.json
```

地图预处理：

```sh
./build/tools/map_preprocess/map_preprocess \
  --input N30E120.hgt \
  --input-format srtm-hgt \
  --output terrain.tile \
  --nodata-fill 0 \
  --index-output terrain_index.bin
```

## 8. 改代码前先定位模块

| 任务 | 优先看 |
|---|---|
| 协议字段、CRC、日志解码 | `common/include/common/protocol.h`, `common/src/packet.c` |
| JSON 配置读取 | `common/src/config.c`, `environment_sim/src/env_app.c`, `flight_control_sim/src/fc_app.c` |
| 地球/坐标/地形 | `environment_sim/src/earth_model.c`, `geo_coordinate.c`, `terrain_model.c`, `map_tile.c` |
| 大气/重力/质量/推进/气动 | `environment_sim/src/*_model.c`, `environment_force_model.c` |
| 6DOF 积分 | `environment_sim/src/missile_plant_6dof.c` |
| 传感器模型 | `environment_sim/src/sensor_*.c` |
| 故障注入 | `environment_sim/src/fault_injection.c` |
| 飞控状态机/保护 | `flight_control_sim/src/fc_modes.c`, `safety_monitor.c`, `fc_app.c` |
| PNG 制导 | `flight_control_sim/src/guidance_png.c`, `guidance_manager.c` |
| 自动驾驶仪/命令管理 | `flight_control_sim/src/autopilot.c`, `command_manager.c` |
| 多实例 | `tools/instance_manager/src/main.c` |
| 回放/比较/批量 | `tools/replay`, `tools/compare_logs`, `tools/batch_stats`, `tools/batch_runner` |

## 9. 常见修改流程

### 修改环境模型

1. 先加或改环境单元测试：`environment_sim/tests/environment_tests.c`。
2. 修改对应 `environment_sim/src/*`。
3. 跑：

```sh
CCACHE_DISABLE=1 cmake --build build
CCACHE_DISABLE=1 ctest --test-dir build -R environment_tests --output-on-failure
CCACHE_DISABLE=1 ctest --test-dir build -R closed_loop_test --output-on-failure
```

### 修改飞控逻辑

1. 先加或改 `flight_control_sim/tests/fc_tests.c`。
2. 修改 `flight_control_sim/src/*`。
3. 跑：

```sh
CCACHE_DISABLE=1 cmake --build build
CCACHE_DISABLE=1 ctest --test-dir build -R flight_control_tests --output-on-failure
CCACHE_DISABLE=1 ctest --test-dir build -R closed_loop_test --output-on-failure
```

### 修改协议或日志

1. 同时检查环境端、飞控端、工具和测试。
2. 跑完整默认测试：

```sh
CCACHE_DISABLE=1 cmake --build build
CCACHE_DISABLE=1 ctest --test-dir build --output-on-failure
```

## 10. 不变量和红线

- 不直接发送 C 结构体内存，必须走固定小端 encode/decode。
- 不让实例之间共享可变运行时状态。
- 不让环境主循环做不可控动态分配。
- 配置、网络和浮点输入必须校验。
- 固定随机种子运行必须可复现。
- LOCKSTEP 下不要破坏一帧传感器对应一帧控制指令的节拍。
- 环境真值主坐标是 ECEF；配置和地图输入是 LLA/WGS-84。

## 11. 当前最值得补的能力

按上手收益排序：

1. `fc_internal_log.bin` 转 CSV/JSON。
2. 小型真实 DEM fixture、`map_manifest.json` 和真实 DEM LOS 回归。
3. 验证报告生成：汇总 summary、diagnostics、compare_logs、batch_stats。
4. 气动表 v2：高度、舵偏维度和容差基准。
5. 非锁步通信故障：突发丢包、乱序、抖动和恢复。
6. 长测分档：short / medium / long，便于本地和 CI 使用。
