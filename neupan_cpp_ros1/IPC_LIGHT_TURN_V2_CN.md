# FW-OMNI-LIGHT-TURN-V2 工控机导入与仿真

本次压缩包为 `neupan_omni_light_turn_v2_noetic_bundle_20261028.tar.gz`，适用于 Ubuntu 20.04 / ROS Noetic。它包含 ROS1 源码、当前 C++ 核心、FW-mini DUNE 权重、地图、RViz 配置、参数快照、调参结果图，以及固定版本 OSQP / OsqpEigen / QDLDL 源码。

**本分发包默认使用 `FW-OMNI-LIGHT-TURN-V2`**：T10/PAN2、参考速度 0.15 m/s、`control_change_weight=[16,0.5]`、`max_speed=[0.18,2*pi]`、`max_acce=[0.2,0.8]`。`config/fw_mini_omni.yaml` 是这套参数的副本，`sim_complete.launch` 默认读取该文件；原完整配置仍在 `config/profiles/ros1/fw_mini_omni_full_v1.yaml`。

开发机回归为 19/19 有路场景通过、另有三张固定无路图，最长连续低速 0.7 s，平均规划耗时约 15.93 ms。工控机需重新编译和实测，本包不带开发机二进制。以下流程只运行虚拟机器人，不需要启动 CAN、底盘驱动、雷达或 FAST-LIO。

## 1. 复制、校验、解压

把压缩包及同名 `.sha256` 文件一起复制到工控机 `~/work`。在新的终端执行：

```bash
cd ~/work
sha256sum -c neupan_omni_light_turn_v2_noetic_bundle_20261028.tar.gz.sha256
tar -xzf neupan_omni_light_turn_v2_noetic_bundle_20261028.tar.gz
cd ~/work/neupan_omni_light_turn_v2_noetic_bundle_20261028
sha256sum -c MANIFEST.sha256
```

校验应全部显示 `OK`。若同名解压目录已存在，先将旧目录改名保留，避免混合版本。

## 2. 编译本次版本

使用新的 `~/neupan_omni_light_turn_build` 和 `~/neupan_omni_light_turn_ws`，保留此前的工作空间。在新终端执行：

```bash
cd ~/work/neupan_omni_light_turn_v2_noetic_bundle_20261028/neupan_cpp_ros1
NEUPAN_OMNI_BUILD_ROOT="$HOME/neupan_omni_light_turn_build" \
NEUPAN_OMNI_WS="$HOME/neupan_omni_light_turn_ws" \
  bash tools/build_ipc.sh all
source ~/neupan_omni_light_turn_build/activate.bash
readlink -f "$(rospack find neupan_cpp_ros1)"
```

最后一行应指向本次解压目录的 `neupan_cpp_ros1`。工作空间的 `src/neupan_cpp_ros1` 是指向该目录的链接，也属于正常情况。以后每个新终端都加载本次 `activate.bash`；测试期间不用再次 source 旧工作空间。

脚本优先复用 `~/.local/neupan-noetic` 或 `/usr/local` 的依赖；缺失时用包内源码编译到新构建目录。源码编译 OSQP 需要 CMake 3.18+ 的 3.x，脚本优先查找 `/opt/cmake-3.22.6/bin/cmake`。原先已成功编译的工控机通常可以直接复用依赖。

若缺少系统开发或绘图库，再安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
  libgtest-dev python3-yaml python3-rospkg python3-numpy python3-matplotlib \
  ros-noetic-rviz
```

ROS 基础消息、roscpp、rospy、tf2_ros 采用现有 Noetic。无需 PyTorch、cvxpy 或原厂底盘消息。

## 3. 验证参数选择

```bash
source ~/neupan_omni_light_turn_build/activate.bash
python3 - <<'PY'
import os,yaml
from pathlib import Path
p=Path(os.environ['NEUPAN_OMNI_PACKAGE_ROOT'])
cfg=yaml.safe_load((p/'config/fw_mini_omni.yaml').read_text())
frozen=yaml.safe_load((p/'config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml').read_text())
assert cfg==frozen, '默认配置与冻结 V2 不同，请核对是否手动修改过'
print('FW-OMNI-LIGHT-TURN-V2')
print('T / PAN:',cfg['receding'],cfg['pan']['iter_num'])
print('control_change_weight:',cfg['adjust']['control_change_weight'])
PY
```

预期为 `10 2` 和 `[16.0, 0.5]`。

## 4. 全场景核心仿真

```bash
source ~/neupan_omni_light_turn_build/activate.bash
cd "$(rospack find neupan_cpp_ros1)"
bash tools/validate_light_turn.sh core \
  "$HOME/neupan_sim_results/light_turn_v2_core_$(date +%Y%m%d_%H%M%S)"
```

脚本显式选择冻结的 V2 文件，先运行 22 张地图，再检查原验收及 **连续低速≤1.5 s**。预期：19/19 有路场景到达和通过；三张全局无路图保留为拒绝，不算控制器通过。开发机最长低速 0.7 s 只是本次对照结果，工控机记录以复测为准。

每次使用新时间戳目录。结果包含：

- `acceptance.json`：本次 1.5 s 验收结论。
- `core/summary.json`：每张地图的耗时、偏差、净距、jerk、低速时间。
- `core/*.png`、`*.csv`、`maps/`：四联图、轨迹和地图。

先单独验证连续急弯，可将命令中的 `core` 换成 `acute`，并使用另一个新结果目录。额外基本回归仍可执行 `bash tools/validate_ipc.sh core 新目录`；其中 omni 使用本包默认 V2，diff 是历史回归，不能代替上述完整测试。

## 5. RViz 交互走廊

```bash
source ~/neupan_omni_light_turn_build/activate.bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  output:="$HOME/neupan_sim_results/light_turn_v2_rviz_$(date +%Y%m%d_%H%M%S)"
```

本包已将该 launch 的默认配置设为 `config/fw_mini_omni.yaml`，即 V2。RViz 使用 **2D Nav Goal** 点选终点，目标话题为 `/neupan_sim/goal`；**2D Pose Estimate** 可设置初始位置和朝向。Fixed Frame 为 `neupan_sim/map`。小车保持车身朝向，靠改变平移方向绕障。

正常 Ctrl+C 退出后，结果在指定目录的 `summary.json`、`trajectory.csv`、`planner.yaml` 等文件中；本次使用的参数以保存的 `planner.yaml` 为准。

## 6. RViz 自动连续急弯与硬直角

连续急弯：

```bash
source ~/neupan_omni_light_turn_build/activate.bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  sim_env_config:=stress/short_acute_zigzag_scenario.yaml \
  auto_goal:=true \
  output:="$HOME/neupan_sim_results/light_turn_v2_rviz_acute_$(date +%Y%m%d_%H%M%S)"
```

连续两个硬直角，将场景改为 `stress/hard_90_double_scenario.yaml`；带墙直角为 `stress/hard_90_walled_scenario.yaml`；同障碍物绕行为 `stress/bar_auto_180_scenario.yaml`。每次换一个输出目录。

使用另一个参数快照时显式传入 `config_file:=完整路径`。本包默认入口和冻结 V2 文件是独立副本：日常修改默认入口会影响 RViz；上面的完整核心验证始终使用冻结文件，以便复现。

## 7. 报告和参数名

- [本次调参结果与曲线](neupan_cpp_ros1/LIGHT_TURN_TUNING_CN.md)
- [配置编号和修改位置](neupan_cpp_ros1/CONFIG_PROFILES_CN.md)
- [本次开发机对照结果](neupan_cpp_ros1/test/results/light_turn_20261028/)
- `SOURCE_VERSIONS.json` / `MANIFEST.sha256`：依赖版本、默认参数与所有文件哈希。

报告中旧默认入口的描述反映调参时的开发机状态；本分发包以本说明及 `SOURCE_VERSIONS.json` 为准，默认已改为 V2。原始核心算法、模型及各版本参数快照保持一致。此次包用于核心/RViz 仿真，档位 7 的实车 CAN 桥接仍是独立的后续工作。
