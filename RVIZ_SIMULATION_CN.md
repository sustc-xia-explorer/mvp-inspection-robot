# FW-mini 的 ROS1 / C++ NeuPAN 交互仿真

这套入口对应原来的 `ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_corridor.yaml` 操作方式。规划器实际运行本仓库的 `neupan_cpp_ros1/neupan_node`，加载你的 `fw_mini.bin`；环境、扫描和理想 diff 虚拟底盘由 `scripts/rviz_sim.py` 提供。

## 更新工控机

将 `neupan_rviz_simulation_update.tar.gz` 复制到工控机 `~/work/`。补充包不包含权重，也不覆盖 `config/fw_mini.yaml`，保留你已训练、导出的 FW-mini 模型。新增控制参数在 `config/fw_mini_sim.yaml`。

```bash
cd ~/work/neupan_noetic_bundle/neupan_cpp
tar -xzf ~/work/neupan_rviz_simulation_update.tar.gz -C .
sudo apt install ros-noetic-rviz python3-yaml
bash tools/build_noetic.sh all
source ~/neupan_noetic_build/activate.bash

roslaunch neupan_cpp_ros1 sim_complete.launch \
  sim_env_config:=scenario_corridor.yaml
```

这里需要重新运行构建，让 catkin 安装新增脚本和 RViz 配置。`models/fw_mini.bin` 应已经存在；若位于别处，在 launch 后添加 `dune_checkpoint:=/绝对路径/fw_mini.bin`。模型的 footprint 元数据必须匹配 0.495 × 0.360 m 的配置。启动日志会检查元数据；旧默认 `diff_default.bin` 不适用于这里。

若旧补充包在构建阶段报 `test_replay` 的 `mean_ms < 5.0` 失败，请更新本补充包后重跑构建。该断言是原开发机的耗时门限；新版保留数值回归，将耗时默认只作报告，显式性能预算的设置见 `SIMULATION_VALIDATION_CN.md`。不要跳过全部 ctest 或反复重跑直到耗时偶然低于 5 ms。

launch 会自行启动所需 ROS master（已有 master 时复用）。这套仿真按墙钟运行，需要 `/use_sim_time=false`；不需要另开 FAST-LIO2、雷达或底盘驱动。ROS1 版本使用 `rviz`，并使用独立的 `neupan_sim/map` 等坐标系。

## 在 RViz 中操作

1. 等待看到走廊、蓝色车体和红色扫描点。初始位姿为 `(-4, 0, 0)`，机器人等待目标。
2. 点击工具栏 **2D Nav Goal**，在可通行区域按下并拖动，松开设定终点。先试 `(-1.6, 0)`，可以观察绕过第一堵墙。
3. 再试 `(4.4, -0.5)`，观察连续绕过四处障碍。绿线是 A* 全局参考路径，橙线是 C++ NeuPAN 预测轨迹，蓝线是实际行驶轨迹。
4. **2D Pose Estimate** 用于重新放置起点和朝向；重设后停止并等待新的终点。放在障碍内的起点会被拒绝。

2D Nav Goal 当前以位置作为抵达目标，拖动箭头不保证最终航向精确到该角度。障碍内或膨胀区内的目标会被拒绝；不存在路径时会明确提示，虚拟车停止。

也可以自动发送默认终点：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  sim_env_config:=scenario_corridor.yaml auto_goal:=true
```

无显示器时运行同一套 ROS1 闭环：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  use_rviz:=false auto_goal:=true
```

要从另一台装有 ROS Noetic 的电脑看 RViz，可在那里连接工控机的 ROS master，再启动同一个 `rviz/fw_mini_sim.rviz` 配置；ROS 网络需能双向访问。

## 场景、车体与参数

默认地图移植自本机 ROS2 的 `ddr_minimal_sim/config/maps/corridor_neupan.yaml`，保留四处内障碍，走廊墙改到 `y=±1.5 m`，墙内净宽 2.8 m。原 ROS2 场景的虚拟车长宽是 0.322 × 0.22 m；这里使用 FW-mini 的 0.495 × 0.360 m。不能直接把小车图示尺寸继续用于 FW-mini。

完全相同的原始地图也已保留：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  sim_env_config:=scenario_corridor_original.yaml
```

原图的走廊墙在 `y=±1 m`，墙内净宽 1.8 m，首个内障碍上、下方各只有约 0.45 m 间隙。这里 A* 使用车体外接圆半径约 0.306 m，加 0.04 m 余量及网格覆盖范围，原图完整穿越会报告无路。这是本全局规划器的保守判定，不能由此断言真实全向底盘在任何姿态下都无法通过，也不能减小显示车体来绕过检验。

可编辑文件：

| 文件 | 用途 |
| --- | --- |
| `config/scenario_corridor.yaml` | 起点、默认终点、传感器外参、扫描频率与网格分辨率 |
| `config/maps/corridor_fw_mini.yaml` | 墙体和内障碍的几何位置 |
| `config/fw_mini_sim.yaml` | 交互仿真的控制参数 |
| `rviz/fw_mini_sim.rviz` | 显示项目、颜色和目标话题 |

`fw_mini_sim.yaml` 与原回归基线 `fw_mini.yaml` 分开。新配置使用 15 步预测、相同 0.1 s 步长、位置/朝向权重 `[6,6,0.5]`、`d_min=0.05`、`d_max=0.15`、`eta=1`。控制频率为 10 Hz；修改步长时须同步调整 launch 的 `control_rate`。车体、DUNE 权重、最大速度仍使用 FW-mini 配置。

`close_threshold=0` 使路径索引在前向搜索范围内查找最近点，关闭“找到半径内第一个点就提前退出”。本次平滑走廊路径在旧索引参数下会停滞，改为最近点搜索后通过。抵达距离仍是 0.10 m，没有扩大抵达阈值。

新配置用于调试可行的仿真闭环。它改变了控制参数和参考路径，不能把它与旧基线的差异算作 C++ 移植本身的收益。7 场景回归中新配置通过 6 个，硬直角 `turn_left` 仍失败；平滑参考路径的成功不等于解决了所有转弯情形。

## 数据流和记录

```text
RViz /neupan_sim/goal
  → 带车体膨胀的 A* + 检查后的转角平滑
  → /neupan_sim/initial_path
  → 实际 ROS1 C++ NeuPAN + fw_mini.bin
  → /neupan_sim/cmd_vel
  → 理想 diff 虚拟底盘 → TF、odom、LaserScan → NeuPAN
```

全局规划器绕过地图中的静态障碍；NeuPAN 使用扫描做局部优化和避障。这个演示验证两者的组合，不是“只给一条穿墙直线、NeuPAN 自动完成全局寻路”的测试，也没有验证动态障碍。仿真只有 `vx` 和 `wz`，尚未验证横移或全向运动模型。

所有输入和速度话题都在 `/neupan_sim/` 下，TF 帧使用 `neupan_sim/` 前缀。初始扫描和 TF 尚未就绪时，规划节点会短暂打印等待警告。命令超过 0.5 s 未更新时虚拟车停止。

仿真独立计算矩形车体与障碍的真实距离。接触障碍时车体变红、记录碰撞并冻结，需重新设置起点恢复；不能把冻结当成规划器成功避障。虚拟底盘执行规划输出，不模拟轮胎、转向切换、CAN、Mid-360S 三维扫描或实际加速度响应。

默认保存到 `~/neupan_sim_results/rviz_corridor`，重名自动追加时间戳。包含 `trajectory.csv`、每次设定目标的参考路径、参数/地图快照、模型哈希和退出时的 `summary.json`。CSV 同时记录原始指令与执行指令，便于区分到达、过期停车和碰撞冻结。按 Ctrl+C 正常退出后查看汇总。

## 验证方法

不依赖 ROS 的地图、路径和车体几何检查：

```bash
python3 -m unittest discover -s neupan_cpp_ros1/test -p test_rviz_world.py
python3 tools/validate_rviz_sim.py \
  --binary ~/neupan_noetic_build/libneupan/neupan_sim_backend \
  --output ~/neupan_sim_results/rviz_core_check
```

已在开发机用实际 FW-mini 权重完成加宽走廊核心闭环：77.3 s 仿真时间到达，终点误差 0.0204 m，最小真实车体间距 0.1640 m，无碰撞。开发机隔离 ROS1 1.15.14 环境还实际运行了同一个 launch（`use_rviz=false`），先检查首个障碍，再完成四处障碍全程：约 78.5 s 墙钟、终点误差 0.02036 m、最小真实间距 0.16404 m、无碰撞；同时检查无目标停车、障碍内目标拒绝、TF/扫描/轨迹、话题隔离和重设起点。完整测试结果保存在 `work/neupan_sim_results/rviz_ros_full_corridor_20261019/validation.json`，其中先完成目标再测试起点重设，故该次 `run/summary.json` 的最终位置已回到起点。开发机为 Ubuntu 22.04；工控机 Ubuntu 20.04/Noetic 与 RViz 图形界面的最终检查仍需执行上面的启动命令。

完整工控机原始结果分析见 `IPC_RESULTS_ANALYSIS_CN.md`。原基线的直角终点不收敛、未完成的障碍场景以及实际输出加速度问题仍应单独验收，当前结果不能作为实车已经可以直接循迹的结论。
