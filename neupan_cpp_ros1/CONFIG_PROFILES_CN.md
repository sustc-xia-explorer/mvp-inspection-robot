# FW-mini 参数配置命名与修改位置

成套规划参数统一归档到 **`config/profiles/`**。编号、中文名、运行核心、冻结参数文件与所有原始来源，见 [配置总表](config/profiles/README.md)、[机器可读索引](config/profiles/index.yaml) 和 [CSV 来源映射](config/profiles/all_profiles.csv)。原文件保留，地图与场景不混作规划器配置。

## 日常使用的名称

| 编号 | 中文名称 | 文件位置，相对本包 |
|---|---|---|
| `FW-OMNI-FULL-V1` | 本地全向完整平滑版 | `config/profiles/ros1/fw_mini_omni_full_v1.yaml` |
| `FW-OMNI-LIGHT-V1` | 本地全向小计算基准 | `config/profiles/ros1/fw_mini_omni_light_v1.yaml` |
| `FW-OMNI-LIGHT-TURN-V2` | 本地全向小计算急弯调优版 | `config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml` |
| `FW-OMNI-CORRIDOR-V1` | 本地全向旧走廊平滑版 | `config/profiles/ros1/fw_mini_omni_corridor_v1.yaml` |
| `FW-OMNI-LEGACY-V0` | 本地全向初版 | `config/profiles/ros1/fw_mini_omni_legacy_v0.yaml` |
| `FW-DIFF-BALANCED-ARCHIVE` | FW 差速均衡归档版 | `config/profiles/archive/fw_mini_diff_balanced_archive.yaml` |
| `UP-FW-OMNI-BASE-V1` | 上游全向 FW 初适配版 | `config/profiles/upstream/fw_mini_omni_base_v1.yaml` |
| `UP-FW-OMNI-TUNED-V2` | 上游全向 FW 调参版 | `config/profiles/upstream/fw_mini_omni_tuned_v2.yaml` |

`FW` 表示机器人；`OMNI/DIFF` 表示模型；`FULL/LIGHT` 区分完整与小计算预算；`TURN` 表示本轮连续急弯优化；`V1/V2` 是冻结参数版本。其余诊断、参考移植、示例和全部试验参数也有编号，完整列在总表和索引中。试验使用 `TRIAL-运行核心-参数内容哈希`，旧试验名可在 `source_files` 搜索。

本轮急弯候选的结果见 [LIGHT_TURN_TUNING_CN.md](LIGHT_TURN_TUNING_CN.md)。它属于当前本地 ROS1 核心，和前一轮“上游原核心调参”是两件事。

## 修改哪一份、启动时选哪一份

本轮建议编辑并显式选择：

```text
config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml
```

工控机复核完成后，默认入口 **`config/fw_mini_omni.yaml` 已选择 `FW-OMNI-LIGHT-TURN-V2`，T10/PAN2**；原完整参数保存在 `config/profiles/ros1/fw_mini_omni_full_v1.yaml`。带版本号文件是冻结快照，不与旧入口双向自动同步。改了命名文件，就把它传给运行入口；实际运行参数以传入的 `config_file` 为准。

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  config_file:="$(rospack find neupan_cpp_ros1)/config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml"
```

复现连续急弯场景：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  sim_env_config:=stress/short_acute_zigzag_scenario.yaml \
  config_file:="$(rospack find neupan_cpp_ros1)/config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml" \
  auto_goal:=true \
  output:="$HOME/neupan_sim_results/light_turn_v2_acute"
```

`sim_complete.launch` 默认读取 `config/fw_mini_omni.yaml`，即 V2；也可显式传入其他命名快照。上游专用配置带有 `projected_reference`，当前正式 ROS1 核心不解析该键；应先按索引中的 `engine` 选核心，再选配置。`upstream/` 里的文件不是当前 ROS1 核心的等价替代文件。归档 diff 仅用于历史比较。

## 规划器参数的中文名称与含义

| YAML 字段 | 中文名称 | 含义/单位 |
|---|---|---|
| `receding` | 预测步数 | T；乘 `step_time` 得预测时间 |
| `step_time` | 模型时间步长 | s；当前 0.1，对应 10 Hz 规划 |
| `ref_speed` | 参考行进速度 | m/s；不是每帧实际输出的保证值 |
| `collision_threshold` | DUNE 紧急停车距离 | m；基于模型估计距离，不等同于精确车身净距 |
| `robot.kinematics` | 运动模型 | 本地可选 `omni` / `diff` |
| `robot.length/width` | 车身足迹长/宽 | m；必须与 DUNE 模型几何一致 |
| `robot.wheelbase` | 足迹原点偏置参数 | 当前 0 表示居中矩形；这里不是填 FW 物理轴距 |
| `robot.vertices` | 自定义凸多边形足迹 | 上游核心支持；当前本地核心不解析此字段 |
| `robot.max_speed[0]` | 行进速度上限 | m/s；omni 内部控制为有符号速度 |
| `robot.max_speed[1]` | 第二控制分量边界 | omni：世界行进方向角 rad；diff：车身角速度 rad/s |
| `robot.max_acce[0]` | 速度变化率上限 | m/s² |
| `robot.max_acce[1]` | 第二控制分量变化率 | omni：行进方向变化率 rad/s；diff：角加速度 rad/s² |
| `robot.periodic_direction` | 行进方向周期重定位 | 本地 omni 按 2π 周期重定位；上游该版本不解析此键 |
| `ipath.arrive_threshold` | 到达距离门限 | m；到达还受路径索引及本地制动逻辑共同约束 |
| `ipath.arrive_index_threshold` | 终段索引门限 | 点数 |
| `ipath.close_threshold` | 最近点搜索提前退出半径 | m；0 表示不因进入半径而提前退出 |
| `ipath.ind_range` | 最近点前向搜索窗口 | 点数 |
| `ipath.curve_style` | 参考曲线类型 | 当前使用 `line` |
| `ipath.loop` | 循环参考路径选项 | 项目配置保持 false，不作为已验证循环导航功能使用 |
| `ipath.interval` | 输入曲线离散间距 | m；缺省为参考速度×模型步长 |
| `ipath.waypoints` | 初始参考航点 | `[x,y,yaw]` 列表；也可从 ROS Path 接入 |
| `ipath.arc_length_sampling` | 连续弧长参考采样 | 本地功能，减少输入点密度对参考推进的影响 |
| `ipath.projected_reference` | 上游投影参考开关 | 上游专用；不是本地 arc_length_sampling 的同名别名 |
| `ipath.turn_rate` | 参考路径转向速度包络 | rad/s；与曲率共同限制参考速度，不是直接设置输出方向速度 |
| `ipath.turn_window` | 曲率估计窗口 | m |
| `ipath.reference_acceleration` | 参考速度包络加速度 | m/s²；提前减速/加速的参考整形参数 |
| `ipath.minimum_turn_speed` | 拐弯参考速度下限 | m/s；只限制参考，不保证实际速度不低于它 |
| `pan.iter_num` | PAN 最大迭代次数 | 次 |
| `pan.iter_threshold` | PAN 收敛门限 | 内部差异判据；不是“允许偏差多少米” |
| `pan.dune_max_num` | DUNE 障碍点上限 | 输入障碍点下采样数量 |
| `pan.nrmp_max_num` | NRMP 障碍约束点数 | 每个预测时刻使用的近障碍点数 |
| `pan.dune_checkpoint` | DUNE 模型路径 | 可由节点的 dune_checkpoint 参数覆盖；几何必须匹配 |
| `adjust.q_s` | 状态跟踪缩放 | 代价实际按 q_s² 加权；omni 跟踪 x/y，不跟踪车身 yaw |
| `adjust.p_u` | 速度跟踪缩放 | 代价实际按 p_u² 加权 |
| `adjust.control_change_weight` | 控制变化代价 | 本地 `[速度变化代价, 行进方向变化代价]`；含相对实际首步控制的变化 |
| `adjust.eta` | 净距奖励系数 | 对优化净距变量的奖励 |
| `adjust.d_min/d_max` | 优化净距变量范围 | m；因存在松弛变量，不能直接当作实际最小净距保证 |
| `adjust.ro_obs` | 避障松弛惩罚 | 越大越不愿用松弛放宽避障约束 |
| `adjust.bk` | 标称轨迹保持代价 | 对偏离标称状态轨迹的二次惩罚 |

## 其他参数层放在哪里

- 全局地图、起终点、激光与全局路径安全边距：`config/scenario_corridor.yaml` 和 `config/stress/*_scenario.yaml`；omni 专用全局参数在 `omni_global_planner` 段。
- 场景障碍物几何：`config/maps/` 和 `config/stress/*.yaml`。它们不应填入规划器的 `config_file`。
- ROS 话题、TF frame、扫描/位姿超时、控制频率：`launch/*.launch` 与节点参数。它们不是上述代价参数文件的一部分。
- FW 驱动接口与档位：硬件桥接配置及对应驱动说明；本次调参不更改 CAN 或硬件使能。

索引是某次整理时的快照，包含每份参数的语义哈希与原始来源。重新整理可运行 `tools/catalog_profiles.py`，原配置不会被搬走或删除。

编号冻结的是参数。历史结果还依赖当时的核心版本、模型和场景；`engine` 用于区分运行核心，不能只凭旧 YAML 保证复现历史行为。本轮 V2 报告同时保存核心与模型哈希。
