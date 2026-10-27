# omni 走廊平滑更新与工控机复测（2026-10-27）

> 本文保存上午一轮走廊调参的历史结果。该轮参数已归档到 `config/fw_mini_omni_corridor_20261027.yaml`；当前 `fw_mini_omni.yaml` 已更新为急弯预减速版本，最新参数和跨地图验证见 [TURN_AND_RANDOM_VALIDATION_CN.md](TURN_AND_RANDOM_VALIDATION_CN.md)。下文参数、结果和旧压缩包对应该历史版本。

本次针对贴障、空旷区域低速/停滞和绕障晃动做了多组仿真。保留原地图、0.495×0.360 m 车体和 DUNE 权重，更新全局路径净距与代价、参考轨迹推进、反向参考速度和方向角环绕，并加入控制变化惩罚。默认 `fw_mini_omni.yaml` 已使用推荐参数，diff 配置保持原数值。

## 1. 推荐参数

规划配置：`config/fw_mini_omni.yaml`。

```yaml
receding: 15
step_time: 0.1
ref_speed: 0.15
collision_threshold: 0.08
robot:
  kinematics: omni
  length: 0.495
  width: 0.360
  wheelbase: 0.0
  max_speed: [0.18, 6.283185307179586]
  max_acce: [0.2, 0.8]
  periodic_direction: true
ipath:
  arc_length_sampling: true
  arrive_threshold: 0.10
pan:
  iter_num: 4
  dune_max_num: 100
  nrmp_max_num: 12
  iter_threshold: 0.005
adjust:
  q_s: [4.0, 4.0, 1.0]
  p_u: 1.5
  eta: 1.0
  d_min: 0.16
  d_max: 0.26
  ro_obs: 600.0
  bk: 0.05
  control_change_weight: [2.0, 0.3]
```

全局路径配置：`config/scenario_corridor.yaml` 中的 omni 专用段。

```yaml
omni_global_planner:
  footprint: fixed_heading
  safety_margin: 0.16
  clearance_weight: 6.0
  preferred_clearance: 0.28
  corner_cut: 0.5
  path_spacing: 0.04
```

`max_speed` 第二项是世界行进方向角的数值范围；`2*pi` 给局部优化留出跨越 ±π 的空间，配合 `periodic_direction` 做等价角重标定，不代表车轮要旋转 360°。`max_acce` 两项分别为有符号速度变化率（m/s²）和世界行进方向变化率（rad/s）。车身仍不旋转，ROS `angular.z=0`。

固定朝向的全局规划按照当前车身矩形在障碍物坐标系中的投影做保守膨胀；A* 加入净距代价，捷径和平滑处理也检查净距。该段只用于 omni，diff 继续使用原来的圆形包络规划。

## 2. 修正了哪些问题

### 路径点密度影响速度

原参考轨迹在 `ref_speed*dt` 大于默认采样间距时按点数推进。相同直线如果被插入更密的路径点，会被当作走过了更短的距离：原测试中 1 m 直线路径，点间距 5 mm 时约 20 s 才到达，稀疏路径约 7.2 s。弧长模式按真实路径长度采样并连续投影，四种点间距的复测都约 7.2 s。

### 负速度与正参考速度相互抵消

`[v,phi]` 可用负速度实现同一平移方向。原参考速度始终为正，会对正常反向行驶施加相反代价，某些目标附近形成零速平衡。新弧长模式按照当前行进角与路径切向选择参考速度符号，反向目标可正常收敛。

### ±π 边界导致连续选点后停滞

仅验证单次前向目标不能发现此问题。连续执行前向、反向、再次选点时，旧角度边界可能让行进角卡在 -π，车在距目标约 0.33 m 处停住，求解器仍报告成功。新配置采用周期方向角，重标定实际控制和暖启动序列的等价角；实际 XY 速度不变。方向变化率统计也按周期差计算，避免把 +π/-π 的表示切换误报为巨大转向速率。

### 绕障抖动与离障碍过近

提高全局路径净距，抑制沿障碍边界取最短捷径；增加速度/方向变化惩罚，并把速度变化率从 0.3 降至 0.2 m/s²、方向变化率从 3.0 降至 0.8 rad/s。控制上限从 0.2 降为 0.18 m/s，参考速度仍为 0.15 m/s。修正参考轨迹后，实际通行时间反而缩短。

新弧长参考和正则项均是显式的部署选项，运动学与 ROS 输出仍为原 omni 模型。`fw_mini_omni_legacy.yaml` 与 `scenario_corridor_legacy.yaml` 保留初始配置；关闭新增选项时，原 Python 数值对照和 diff 回放仍通过。

## 3. 结果与适用范围

[前向对比图](test/results/omni_smooth_20261027/compare_forward.png) 给出同地图、同一 50 Hz 虚拟底盘的轨迹、速度、真实车体净距和合成加速度。

对比以同一目标、同一矩形碰撞几何计算，未放宽到达门限，也未缩小车体。初始配置的前向场景为 77.4 s、最小净距 8.70 cm、最大 XY 加速度 0.596 m/s²；推荐配置前向约 68.4 s、最小净距约 20.7 cm、最大 XY 加速度约 0.211 m/s²，jerk 均方根由约 2.427 降至 0.277 m/s³。

完整运行记录在 `test/results/omni_smooth_20261027/`，其中包含候选参数、失败用例和最终用例。最终多目标 ROS1 验证结果在本节末尾的实测表中记录。

这些参数是本地图下的验证推荐值。全局 16 cm 和 NRMP 的 `d_min` 不构成对任意目标、地图、动态障碍和真实执行器的硬净距保证。实际轨迹的净距与平滑度仍需在工控机复测。实车 CAN 档位 7 桥接不属于本更新。

## 4. 工控机导入新包

复制压缩包及校验文件到工控机 `~/work` 后：

```bash
cd ~/work
sha256sum -c neupan_omni_smooth_noetic_bundle_20261027.tar.gz.sha256
tar -xzf neupan_omni_smooth_noetic_bundle_20261027.tar.gz
cd ~/work/neupan_omni_smooth_noetic_bundle_20261027
sha256sum -c MANIFEST.sha256
cd neupan_cpp_ros1
NEUPAN_OMNI_BUILD_ROOT="$HOME/neupan_omni_smooth_build" \
NEUPAN_OMNI_WS="$HOME/neupan_omni_smooth_ws" \
  bash tools/build_ipc.sh all
source ~/neupan_omni_smooth_build/activate.bash
rospack find neupan_cpp_ros1
```

请在新终端使用以上环境。新工作空间为 `neupan_omni_smooth_ws`，保留之前的 `neupan_omni_ws`。若 CMake 不在自动识别的位置，可设置 `NEUPAN_CMAKE=/opt/cmake-3.22.6/bin/cmake`；依赖仍优先复用之前的安装前缀，包中也带有依赖源码。

## 5. 核心与 RViz 复测

标准回归测试：

```bash
source ~/neupan_omni_smooth_build/activate.bash
cd ~/work/neupan_omni_smooth_noetic_bundle_20261027/neupan_cpp_ros1
bash tools/validate_ipc.sh core ~/neupan_sim_results/omni_smooth_core
```

走廊定向用例（前向、反向、中途向左、旋转初始朝向、空旷反向和横移）：

```bash
bash tools/validate_ipc.sh corridor ~/neupan_sim_results/omni_smooth_corridor
```

实际 ROS1 连续三目标闭环，自动启动独立 master，无须人工点选：

```bash
bash tools/validate_ipc.sh ros-smooth ~/neupan_sim_results/omni_smooth_ros
```

人工 RViz 可视化：

```bash
source ~/neupan_omni_smooth_build/activate.bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  config_file:="$(rospack find neupan_cpp_ros1)/config/fw_mini_omni.yaml" \
  output:="$HOME/neupan_sim_results/omni_smooth_rviz"
```

使用 2D Nav Goal 连续选择终点，观察小车保持车身朝向并平移绕障。正常退出后保留 `summary.json`、`trajectory.csv` 和 `goal_*_path.json`。本次 CSV 还记录求解状态、规划耗时和目标距离，便于区分求解失败、到达制动与真实停滞。


## 6. 最终实测表

以下为同一本机、同地图、同一 50 Hz 理想底盘的核心闭环对比：

| 配置 | 前向耗时 s | 最小真实净距 cm | 最大 XY 加速度 m/s² | jerk RMS m/s³ |
| --- | ---: | ---: | ---: | ---: |
| 初始参数 | 77.4 | 8.70 | 0.596 | 2.427 |
| 仅修正弧长参考 | 65.6 | 8.88 | 0.611 | 1.983 |
| 弧长参考＋全局净距代价 | 67.9 | 19.13 | 0.489 | 0.998 |
| 推荐完整配置 | 68.4 | 20.66 | 0.211 | 0.277 |

推荐配置的速度总变化量下降约 88.7%，jerk RMS 下降约 88.6%；前向通行时间缩短约 11.6%。选择 15 步预测时域作为平滑性与计算量的折中；20 步候选更耗时，并非本次默认选择。

最终真实 ROS1 连续选点采用同一个节点、同一个虚拟机器人，保留历史控制状态：

| 目标序列 | 用时约 s | 到达采样误差约 cm | 结果 |
| --- | ---: | ---: | --- |
| 前向终点 `[4.4,-0.5]` | 72.8 | 5.35 | 到达 |
| 返回起点 `[-4,0]` | 71.3 | 3.70 | 到达 |
| 再选左侧目标 `[-1.65,0.65]` | 22.5 | 1.96 | 到达 |

三目标全程零碰撞，最小真实车体净距 **20.84 cm**，车身 yaw 保持 0；最大实际 XY 合成加速度 **0.246 m/s²**，最大标量速度变化率 **0.2 m/s²**，最大方向变化率 **0.8 rad/s**。远离终点的最长低速段约 **0.36 s**，没有持续 3 s 的停滞，日志没有 stalled 或连续求解失败警告。

另做了核心途中换向：运动至 30 s 时直接重规划返回起点，正常到达，最小净距 20.66 cm，最长低速段 0.2 s；没有通过重启规划器清除控制历史。

标准 C++ 回归为 5/5 组、Python 检查为 19/19 项；omni 和 diff 的 6 个标准闭环场景均通过。原 Python QP 数值参考仍使用未启用扩展项的参数验证。

本机 ROS1 记录的规划耗时 P95 约 **63.6 ms**，最大约 **107.9 ms**；有个别帧超过 100 ms，本更新不保证每一帧都满足 10 Hz 截止时间。这是本机隔离 ROS1 的结果，尚未作为工控机的新参数实测结果。工控机复测时请同时保留 `planner_ms` 列，以便检查计算余量。

详细数据：[comparison.json](test/results/omni_smooth_20261027/comparison.json)、[ROS1 三目标结果](test/results/omni_smooth_20261027/FINAL_ROS1_SUMMARY.json)、[对比图](test/results/omni_smooth_20261027/compare_forward.png)。
