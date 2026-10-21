# 到达距离门限 0.10 m / 0.20 m 对照

本文记录调参前两套配置的门限实验。后续 `fw_mini_balanced.yaml` 已在保持 0.10 m 到达门限时通过原 7 场景；当前仿真与实机入口均默认加载该候选，详见 [调参报告](PARAMETER_TUNING_CN.md)。下文“RViz 参数”指保留的旧 `fw_mini_sim.yaml`。

2026-10-21 在开发机重新编译当前 C++ 核心，用实际 `fw_mini.bin` 完成两套参数各 7 个场景、以及完整四障碍走廊的成对闭环测试。只修改 `ipath.arrive_threshold`，原始配置和历史结果均保留。

## 结果

| 配置 | 到达门限 0.10 m | 到达门限 0.20 m | 仍未通过的场景 |
| --- | --- | --- | --- |
| 原始 `fw_mini.yaml` | 5/7，71.4% | 6/7，85.7% | `corridor` |
| RViz `fw_mini_sim.yaml` | 6/7，85.7% | 6/7，85.7% | 硬直角 `turn_left` |

7 个场景中，`blocked_wall` 的成功条件是接近封路障碍后安全停车、且不误报到达。因此上表是场景验收通过比例；只统计其余 6 个有路导航场景时，原始配置由 4/6 提高到 5/6，RViz 配置仍为 5/6。每种条件下每个场景运行一次，不能将该比例解释为多次随机实验证明的可靠性。

| 场景 | 原始参数 0.10 m | 原始参数 0.20 m | RViz 参数 0.10 m | RViz 参数 0.20 m |
| --- | --- | --- | --- | --- |
| `straight` | 通过 | 通过 | 通过 | 通过 |
| `heading_north` | 通过 | 通过 | 通过 | 通过 |
| `turn_left` | 失败，终点误差 0.13646 m | 通过，终点误差 0.13869 m | 失败，终点误差 0.98162 m | 失败，终点误差 0.98162 m |
| `sparse_stop` | 通过 | 通过 | 通过 | 通过 |
| `corridor` | 失败，终点误差 2.03975 m | 失败，终点误差 2.03975 m | 通过 | 通过 |
| `detour` | 通过 | 通过 | 通过 | 通过 |
| `blocked_wall` | 通过安全停车验收 | 通过安全停车验收 | 通过安全停车验收 | 通过安全停车验收 |

原始参数的直角场景已完成转弯，但最终存在约 0.136 m 横向误差，不能满足原先 0.10 m 的到达半径。改为 0.20 m 后，20.2 s 首次报告到达，停车验收在 21.2 s 完成，最终位置约 `(1.13676, 0.97696)`，目标是 `(1, 1)`。通过率提高来自允许该终点偏差，不代表转弯跟踪精度得到改善；其最终误差反而由 0.13646 m 略增到 0.13869 m。

RViz 参数的硬直角场景停在约 `(0.96099, 0.01916)`，离目标仍约 0.982 m，放宽到达半径没有帮助。原始参数的狭窄走廊停滞也不受此次修改影响。

## 完整 RViz 走廊

使用 `scenario_corridor.yaml`、同一个 A* 与转角平滑、同一个扫描及虚拟底盘，重跑 `tools/validate_rviz_sim.py`：

| 到达门限 | 结果 | 仿真时间 | 最终位置误差 | 最小真实车体间距 | 碰撞 |
| --- | --- | --- | --- | --- | --- |
| 0.10 m | 通过 | 77.3 s | 0.020423 m | 0.164045 m | 无 |
| 0.20 m | 通过 | 77.3 s | 0.020423 m | 0.164045 m | 无 |

两次轨迹和最终状态相同。`libneupan/src/initial_path.cpp` 的到达判断同时要求：

1. 到终点的距离严格小于 `arrive_threshold`；
2. 路径索引推进到末尾附近，由 `arrive_index_threshold` 控制。

因此，设置 0.20 m 不等于每次一进入终点周围 0.20 m 就立即停车。当前完整走廊及多数成功场景仍由路径索引条件决定何时报告到达。`close_threshold` 是路径最近点搜索参数，不是此次修改的到达半径。

## 验收口径与验证范围

- 7 场景脚本的外部终点误差验收仍为 `--goal-tolerance 0.15`，没有同时放宽；路径误差、超调、真实间距、速度边界及超时条件均保持不变。
- 对相同轨迹另按终点误差 0.20 m 复核，通过数仍分别为 6/7 和 6/7。失败场景没有报告到达且距离仍很大。
- `collision_threshold=0.03`、DUNE 权重、车体尺寸、优化参数均未修改。到达距离门限与距障碍物的停车判断是不同参数。
- 0.10 m 对照组的全部场景通过状态与最终误差，均精确复现之前保存的原始参数和 RViz 参数结果。
- 当前源码重新构建后的 `ctest` 为 3/3 通过。本次运行的是 ROS1 包使用的同一个 C++ 核心与独立虚拟底盘，没有重新启动 ROS1 接口或 RViz 图形界面；也不是新工控机或实车结果。
- 本次不据耗时结果评价轻量化加速比，也没有重新测试 ROS2 Python 包或全向运动模型。

## 保存位置与复现

完整数据位于开发机：

```text
/home/xiapilang/work/neupan_sim_results/arrival_threshold_20261021/
├── experiment.json                  # 模型、源码、配置及可执行文件哈希
├── configs/
│   ├── fw_mini_arrive_020.yaml
│   └── fw_mini_sim_arrive_020.yaml
├── core_baseline_010/                # 每个场景的轨迹、汇总、参数快照
├── core_baseline_020/
├── core_rviz_010/
├── core_rviz_020/
├── rviz_corridor_010/                # 完整四障碍走廊
├── rviz_corridor_020/
├── comparison.csv
├── comparison.json
├── turn_left_comparison.png
└── reproduce.py
```

每个结果目录均含 `summary.json`，7 场景目录另含 `summary.csv`。原始参数和模型的 SHA256 与实验清单一同保存。试验配置只修改到达半径，正式配置仍为 0.10 m。

在工控机复测时，以该机的代码、模型和构建路径为准，例如复制上述试验配置到工控机后执行：

```bash
source ~/neupan_noetic_build/activate.bash
rosrun neupan_cpp_ros1 simulation_test.py \
  --backend core --cases all \
  --config /绝对路径/fw_mini_arrive_020.yaml \
  --goal-tolerance 0.15 \
  --output ~/neupan_sim_results/core_arrive_020
```

RViz 参数对照使用 `fw_mini_sim_arrive_020.yaml`。若要在界面观察同一修改，可通过 launch 的配置参数加载试验文件：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  config_file:=/绝对路径/fw_mini_sim_arrive_020.yaml
```

是否采用 0.20 m，应由任务允许的终点定位误差决定。它能解决原始直角场景的到达判定问题，仍需分别处理硬直角路径停滞、狭窄走廊振荡和实际底盘响应问题。
