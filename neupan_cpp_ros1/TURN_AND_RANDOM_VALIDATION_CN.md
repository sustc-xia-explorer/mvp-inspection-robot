# omni 急弯、同一障碍物绕行及随机室内地图验证

工控机复测已完成，最新实测及复核结论见 [IPC_OMNI_RESULTS_20261027_CN.md](IPC_OMNI_RESULTS_20261027_CN.md)。下文保留开发机调参对照，并与工控机结果区分。

日期：2026-10-27。当前 `config/fw_mini_omni.yaml` 已更新为本报告的统一参数。全部实现、地图、测试和结果位于本 `neupan_cpp_ros1/` 包中。

## 1. 结论与范围

八个专项场景全部通过。十四张固定随机种子的地图中，十一张存在当前全局规划器认可的可行路径，十一张均到达；其余三张在矩形车体膨胀与 16 cm 安全余量下，起终点不连通，全局规划拒绝发出路径。共 22 个场景，19 次有效控制闭环、19 次到达，零碰撞、零控制器跟踪失败。**全部地图到达率是 19/22（86.4%）；有全局可行路径的闭环到达率是 19/19（100%）**，不能混为一谈。

19 次核心闭环中的最小真实车体净距为 17.90 cm，最大路径偏差为 3.99 cm，最大连续低速时间 0.4 s。车体尺寸始终为 0.495×0.360 m、到达门限始终为 0.10 m，未按地图缩小车体或放宽门限。

硬直角控制过冲从约 2.59 cm 降至 0.49 cm；原始折线路径最大跟踪偏差由 2.59 cm 降至 0.92 cm。真实 ROS1 带墙直角复测的过冲约 0.80 cm。路径偏差与过冲是不同指标：前者包含内侧切角，后者只统计越过拐角的外侧距离。

这一版通过了明确列出的仿真条件，不宣称任意地图、任意急弯、真实轮角执行器均已验证。该模型仍是保持车身朝向的平移运动学，未加入轮角伺服延迟、滑移、动态障碍或档位 7 CAN 桥接。

## 2. 原因与修正

仅降低最大速度不能充分处理急弯。此前参考轨迹按固定 `ref_speed` 推进，即使接近折线拐角也继续以相同速度向下一段移动；实际行进角只能以 0.8 rad/s 改变，控制器会在越过拐角后反向纠偏。路径上的几何不连续、参考速度和实际方向变化能力之间存在冲突，不能全部归因于 A*。

本次保留全局地图、膨胀、净距代价和 DUNE 权重，加入**参考轨迹的预减速**：

1. 沿弧长统计局部路径转角，使用 0.16 m 窗口得到平滑的转弯强度。
2. 按目标路径转角速率 0.4 rad/s 限制该处参考速度，折点处参考速度下限为 0.035 m/s。
3. 沿路径前后传播 `v² ≤ v_next² + 2 a Δs`，参考加减速度取 0.10 m/s²，在进入急弯前减速、离开后恢复，并在终点前预减速。
4. 提高位置跟踪权重与控制变化惩罚，减弱“始终追求巡航速度”的代价，减少越过拐角后反复修正。

`minimum_turn_speed` 是参考下限，不是实际速度保证；硬角附近实际仍可能短暂低于 0.02 m/s。极急转弯需要减速。此方案减少非必要的速度起伏，不要求硬直角处保持恒定速度，也没有增加 jerk 硬约束。

`turn_rate: 0` 关闭新速度包络，保留原固定速度参考；新增选项仅允许在 `omni + arc_length_sampling` 下启用。原 Python QP 数值对照、diff 回放继续通过。

## 3. 冻结参数

规划文件为 `config/fw_mini_omni.yaml`。核心改动如下，其余沿用上一轮配置：

```yaml
ref_speed: 0.15
ipath:
  arc_length_sampling: true
  arrive_threshold: 0.10
  turn_rate: 0.4
  turn_window: 0.16
  reference_acceleration: 0.10
  minimum_turn_speed: 0.035
adjust:
  q_s: [6.0, 6.0, 1.0]
  p_u: 0.7
  control_change_weight: [12.0, 1.0]
```

速度上限 `[0.18, 2*pi]`、变化率 `[0.2, 0.8]`、预测 15 步×0.1 s、`d_min=0.16`、`d_max=0.26`、碰撞紧急阈值 0.08 m、全局安全余量 0.16 m 均保持原值。omni 的第二控制量是世界行进方向角，不是 yaw 角速度。`turn_rate` 是参考速度规划目标，真正的方向速率硬限制仍为 `max_acce[1]=0.8 rad/s`。

配置 SHA256：`d2e406ff8e80a86071bd39e218d33ba345ff88831d55275abd75561ed6f6f12e`。全部 22 个最终用例记录的配置散列相同。此前走廊参数保存在 `config/fw_mini_omni_corridor_20261027.yaml`；最初移植参数仍为 `fw_mini_omni_legacy.yaml`。这些是测试范围内的折中推荐参数，不是全局最优性证明。

## 4. 场景与验收方法

- 绕杆：0.24×2.2 m 的同一根竖直障碍，左右两侧互换目标；覆盖自动 A* U 形绕行、指定上侧/下侧绕行、移动 8 s 后切换至另一侧目标。
- 硬直角：直接输入未经圆角处理的折线路径，测试单直角、连续直角及 135° 急折返。L 形带墙通道宽 1.8 m，路径按水平段后竖直段输入，不让全局圆角掩盖控制器问题。
- 随机地图：13 张 8×8 m 正方形房间、1 张 10×6 m 长方形房间，每张包含 5～22 个内部矩形/圆形障碍及四周墙；位置、大小、矩形角度均按固定种子生成，允许障碍接触/重叠。障碍密度不是逐次人工调整的。包含 yaw=90° 的车体坐标转换测试。
- 六张随机地图用于早期诊断；参数冻结后再使用另外八个种子的地图验证，没有针对单张地图修改控制参数。所有生成地图都保留，无路地图不重采样替换。

控制器使用真实 C++ core，10 Hz 求解，虚拟底盘 50 Hz 执行，并将实际执行速度反馈到下一轮优化。激光雷达 360 束、10 Hz、5 m 量程。碰撞/净距按真实矩形车体几何计算，独立于 DUNE 估计。

有效闭环验收条件：到达、零碰撞、最小净距≥0.12 m、最大路径偏差≤0.10 m、连续低速≤2 s、100 ms 差分 XY 加速度≤0.30 m/s²、无求解失败。低速定义为速度<0.02 m/s、距终点>0.20 m、运行超过 2 s。jerk 和速度波纹用于比较平滑性，未设置任意的“全部平滑”布尔标签。速度波纹为扣除 1.1 s 滑动平均后的 RMS，不能与有计划的转弯减速混淆。

## 5. 全部结果

| 场景 | 到达/结果 | 最小净距 cm | 最大路径偏差 cm | 用时 s | jerk RMS m/s³ |
| --- | --- | ---: | ---: | ---: | ---: |
| 同一障碍物自动 U 形绕行 | 通过 | 23.78 | 3.77 | 28.7 | 0.331 |
| 移动中切换至障碍另一侧 | 通过 | 17.90 | 3.68 | 37.2 | 0.261 |
| 从下侧绕杆 | 通过 | 36.92 | 3.99 | 49.7 | 0.294 |
| 从上侧绕杆 | 通过 | 36.92 | 3.97 | 49.7 | 0.301 |
| 空地原始硬直角 | 通过 | 215.26 | 0.92 | 30.7 | 0.177 |
| 连续两个硬直角 | 通过 | 165.26 | 0.90 | 51.3 | 0.177 |
| 短距离 135° 急折返 | 通过 | 215.26 | 0.93 | 50.9 | 0.232 |
| 1.8 m 宽带墙 L 形通道 | 通过 | 64.78 | 0.92 | 63.3 | 0.125 |
| random_1103（6 个内部障碍） | 通过 | 25.42 | 1.11 | 64.3 | 0.146 |
| random_2207（10 个内部障碍） | 通过 | 24.53 | 2.02 | 60.7 | 0.157 |
| random_3301（14 个内部障碍） | 通过 | 24.18 | 1.44 | 73.3 | 0.163 |
| random_4409（18 个内部障碍） | 通过 | 18.41 | 1.38 | 64.4 | 0.165 |
| random_5519（22 个内部障碍） | 通过 | 20.14 | 1.49 | 83.1 | 0.175 |
| random_6619（12 个内部障碍） | 全局无路，未运行控制器 | — | — | — | — |
| random_7103（5 个内部障碍） | 通过 | 19.67 | 1.42 | 61.2 | 0.140 |
| random_8209（9 个内部障碍） | 通过 | 21.58 | 1.42 | 72.9 | 0.116 |
| random_9311（13 个内部障碍） | 通过 | 24.32 | 1.48 | 68.6 | 0.140 |
| random_10427（17 个内部障碍） | 全局无路，未运行控制器 | — | — | — | — |
| random_11549（21 个内部障碍） | 通过 | 19.63 | 3.60 | 61.6 | 0.192 |
| random_12671（11 个内部障碍） | 通过 | 23.07 | 1.47 | 65.9 | 0.120 |
| random_13781（15 个内部障碍） | 通过 | 23.05 | 1.45 | 70.5 | 0.127 |
| random_14891（19 个内部障碍） | 全局无路，未运行控制器 | — | — | — | — |

无路地图为 `random_6619`、`random_10427`、`random_14891`。独立的不带启发式洪泛搜索确认：在**同一保守膨胀网格**上起终点不连通，证据在 `final_core/no_route_proof.json`。这不等同于证明物理车体在所有连续路径下绝对无法通过，也不计为控制器到达。

### 同场景与前一版的对照

| 场景 | 最大路径偏差 cm：原→新 | jerk RMS：原→新 | 用时 s：原→新 |
| --- | ---: | ---: | ---: |
| 同一障碍物自动 U 形绕行 | 4.37 → 3.77 | 0.532 → 0.331 | 27.5 → 28.7 |
| 移动中切换至障碍另一侧 | 4.36 → 3.68 | 0.390 → 0.261 | 37.0 → 37.2 |
| 从下侧绕杆 | 4.58 → 3.99 | 0.356 → 0.294 | 44.5 → 49.7 |
| 从上侧绕杆 | 4.54 → 3.97 | 0.372 → 0.301 | 44.5 → 49.7 |
| 空地原始硬直角 | 2.59 → 0.92 | 0.270 → 0.177 | 27.8 → 30.7 |
| 连续两个硬直角 | 2.60 → 0.90 | 0.286 → 0.177 | 44.8 → 51.3 |
| 短距离 135° 急折返 | 2.77 → 0.93 | 0.429 → 0.232 | 35.2 → 50.9 |
| 1.8 m 宽带墙 L 形通道 | 2.56 → 0.92 | 0.196 → 0.125 | 60.5 → 63.3 |

急弯越多，预减速付出的用时越明显；短急折返从 35.2 s 增至 50.9 s，换来最大路径偏差约减小 67%、jerk RMS 约减小 46%。速度总变化量并非所有场景都下降，因为新方案主动减速后再恢复，完整指标保存在 JSON 中。

上一轮走廊三目标回归仍全部到达：前向 69.7 s、反向 69.1 s、第三目标 20.2 s，最小净距约 18.8 cm，无连续停滞；相比上一轮约 20.4～20.8 cm 的最小净距略小，未宣称所有指标同时最优。

### 真实 ROS1 接口复核

使用独立 ROS master、实际 `neupan_node`、`sim_complete.launch` 中的扫描/TF/反馈节点；GUI 关闭，和 RViz 使用相同的数据流。测试在当前电脑的隔离 ROS1 环境执行，并非工控机实测。

| ROS1 场景 | 到达 | 最小净距 cm | 最大路径偏差 cm | 最大实际 XY 加速度 m/s² | 最长低速 s |
| --- | --- | ---: | ---: | ---: | ---: |
| 同一障碍物 U 形绕行 | 是 | 23.85 | 4.66 | 0.242 | 0.72 |
| 带墙硬直角 | 是 | 64.45 | 0.82 | 0.214 | 0.52 |

两场景均零碰撞、零跟踪求解失败，body yaw 保持不变。带墙直角的外侧过冲约 0.80 cm。ROS1 求解 P95 约 47～50 ms，本次最大约 66 ms；这是当前电脑结果，不能替代工控机性能测量。ROS1 约 50 Hz 差分的 jerk 与核心 10 Hz 差分指标不可直接对比，两种原始采样均保留。

核心 CTest 5/5 组、Python 单元测试 20/20、omni 基本回归 6/6、diff 基本回归 6/6 通过。

## 6. 图表与数据

全部位于 `test/results/omni_turns_20261027/`：

- [专项场景总览](test/results/omni_turns_20261027/plots/targeted_overview.png)
- [全部随机地图总览，包括无路地图](test/results/omni_turns_20261027/plots/random_overview.png)
- [前后参数对比图](test/results/omni_turns_20261027/plots/targeted_comparison.png)
- [自动绕同一障碍物的四联图](test/results/omni_turns_20261027/final_core/bar_auto_180.png)
- [带墙硬直角四联图](test/results/omni_turns_20261027/final_core/hard_90_walled.png)
- [完整汇总与对比 JSON](test/results/omni_turns_20261027/plots/comparison.json)

`final_core/<case>.png` 为每个有效闭环的地图/轨迹、速度分量、净距、加速度/jerk；同目录 CSV 保存原始记录，路径 JSON 和 `maps/` 保存输入。三个无路场景无闭环 CSV，在随机总览中明确标记。`baseline/`、`trial_v1/2/3`、`random_trial_v1/` 保存原配置及调参过程，不与最终结果混用。最终结论以 `final_core/`、两组 `ros1_*`、`corridor_regression/` 为准。

## 7. 工控机编译与复测

将 `neupan_omni_turns_noetic_bundle_20261027.tar.gz` 和 `.sha256` 复制到工控机 `~/work`，新开终端：

```bash
cd ~/work
sha256sum -c neupan_omni_turns_noetic_bundle_20261027.tar.gz.sha256
tar -xzf neupan_omni_turns_noetic_bundle_20261027.tar.gz
cd ~/work/neupan_omni_turns_noetic_bundle_20261027
sha256sum -c MANIFEST.sha256
cd neupan_cpp_ros1
NEUPAN_OMNI_BUILD_ROOT="$HOME/neupan_omni_turns_build" NEUPAN_OMNI_WS="$HOME/neupan_omni_turns_ws"   bash tools/build_ipc.sh all
source ~/neupan_omni_turns_build/activate.bash
rospack find neupan_cpp_ros1
```

默认尝试复用已有 OSQP/OsqpEigen 用户前缀，压缩包也含固定版本依赖源码。可用 `NEUPAN_CMAKE=/opt/cmake-3.22.6/bin/cmake` 指定 CMake。新工作空间不会覆盖旧版；必须使用本次 `activate.bash`，避免运行旧二进制。

完整核心场景复测：

```bash
bash tools/validate_ipc.sh stress "$HOME/neupan_sim_results/omni_turns_ipc"
```

生成相同地图与完整结果。当前脚本将“已确认无路且不发出路径”作为正常的全局结果返回，但该地图的 `pass=false, arrived=false, geometric_no_path=true` 保留，不计入控制器通过率；有效路径上的任何失败仍使脚本返回非零。

在 RViz 自动启动同障碍物 U 形场景：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch   config_file:="$(rospack find neupan_cpp_ros1)/config/fw_mini_omni.yaml"   sim_env_config:=stress/bar_auto_180_scenario.yaml   auto_goal:=true output:="$HOME/neupan_sim_results/omni_turns_rviz_bar"
```

带墙硬直角仅替换场景和输出目录：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch   config_file:="$(rospack find neupan_cpp_ros1)/config/fw_mini_omni.yaml"   sim_env_config:=stress/hard_90_walled_scenario.yaml   auto_goal:=true output:="$HOME/neupan_sim_results/omni_turns_rviz_90"
```

`config/stress/` 包含全部 22 个场景，可用 `stress/random_5519_scenario.yaml` 等替换。直角/指定绕行场景的 `test_reference_path` / `test_via_points` 仅在初始位姿与默认目标匹配时使用，并先验证路径安全；其他交互目标仍走普通 A*。RViz 的 2D Nav Goal 可继续选点，输出仍完全隔离在 `/neupan_sim/`。

真实 ROS1 单场景自动验收（关闭 GUI）：

```bash
python3 test/ros1_smooth_corridor_test.py --scenario-goal   --scenario config/stress/hard_90_walled_scenario.yaml   --output "$HOME/neupan_sim_results/omni_turns_ros1_90"
```

重新生成前后对比图：

```bash
python3 tools/plot_omni_stress.py   --results "$HOME/neupan_sim_results/omni_turns_ipc/stress_core"   --baseline test/results/omni_turns_20261027/baseline   --output "$HOME/neupan_sim_results/omni_turns_ipc/plots"
```

绘图需要 matplotlib、numpy、PyYAML（Ubuntu 可安装 `python3-matplotlib python3-numpy python3-yaml`）。所有测试命令只启动虚拟机器人；本次未接通实车控制。
