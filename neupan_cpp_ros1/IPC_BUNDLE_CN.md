# 工控机导入与 omni 复测

此包适用于已有 Ubuntu 20.04 / ROS Noetic 的工控机。包含当前 omni/diff ROS1 源码、FW-mini DUNE `.bin` 权重、RViz 地图与显示配置、核心/ROS1 验证脚本、之前的结果，以及 OSQP、OsqpEigen、QDLDL 依赖源码。

包内不含本机编译的二进制文件，需要在工控机重新编译。独立使用 `~/neupan_omni_build` 和 `~/neupan_omni_ws`，保留原来的 `neupan_noetic_build`、`neupan_noetic_ws`。这里的仿真无需启动底盘、CAN、雷达或 FAST-LIO。

## 1. 复制、检查并解压

把以下两个文件一起复制到工控机 `~/work`：

- `neupan_omni_noetic_bundle_20261027.tar.gz`
- `neupan_omni_noetic_bundle_20261027.tar.gz.sha256`

在工控机的新终端执行：

```bash
cd ~/work
sha256sum -c neupan_omni_noetic_bundle_20261027.tar.gz.sha256
tar -xzf neupan_omni_noetic_bundle_20261027.tar.gz
cd ~/work/neupan_omni_noetic_bundle_20261027
sha256sum -c MANIFEST.sha256
```

外部校验应显示 `OK`，内部清单应全部通过。目录已经存在时，先将旧目录改名保留再解压，避免混合两次版本。

## 2. 编译本次 omni 版本

本机已经做过 Noetic 仿真时，通常不需要重新安装系统依赖。如果缺少编译/测试依赖，可以安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
  libgtest-dev python3-yaml python3-rospkg ros-noetic-rviz
```

ROS 基础消息、roscpp、rospy、tf2_ros 使用现有 Noetic 安装。无需安装 PyTorch、cvxpy 或 Python NeuPAN，也不需要原厂底盘消息包。

进入包内执行专用脚本：

```bash
cd ~/work/neupan_omni_noetic_bundle_20261027/neupan_cpp_ros1
bash tools/build_ipc.sh all
source ~/neupan_omni_build/activate.bash
rospack find neupan_cpp_ros1
```

最后一行应指向 **本次解压目录中的 `neupan_cpp_ros1`**（或新工作空间的同一源目录），不能是旧 `neupan_noetic_bundle` 中的版本。

该脚本会：

1. 优先使用已有 `~/.local/neupan-noetic` 或 `/usr/local` 的 OSQP/OsqpEigen。
2. 依赖不存在时，使用包内源码编译到 `~/neupan_omni_build/deps_install`。
3. 编译 `neupan_cpp_ros1/core`，执行全部 5 组 C++ 测试。
4. 在独立 `~/neupan_omni_ws` 中编译 ROS1 节点。
5. 生成本次专用的 `~/neupan_omni_build/activate.bash`。

脚本不调用旧的 `tools/build_noetic.sh`，也不链接外层已归档的差速库。

Ubuntu 20.04 默认 CMake 3.16 可编译核心；从源码编译包内 OSQP 需要 CMake 3.18 以上的 3.x 版本。脚本会优先选择此前安装的 `/opt/cmake-3.22.6/bin/cmake`。若在其他位置，请显式指定，例如：

```bash
NEUPAN_CMAKE=/opt/cmake-3.22.6/bin/cmake bash tools/build_ipc.sh all
```

如果以前把依赖装在其他前缀，可以指定 `NEUPAN_INSTALL_PREFIX=/实际路径`。已有新工作空间若指向其他源码，脚本会停止，保留原内容；此时可用 `NEUPAN_OMNI_WS=$HOME/neupan_omni_ws_20261027` 选择另一个工作空间。

只需要无 ROS 核心时可执行 `bash tools/build_ipc.sh core`，之后使用 `source ~/neupan_omni_build/core_env.bash`。需要 RViz 时仍应完成 `all`。

## 3. 核心仿真再次验证

```bash
source ~/neupan_omni_build/activate.bash
cd ~/work/neupan_omni_noetic_bundle_20261027/neupan_cpp_ros1
bash tools/validate_ipc.sh core ~/neupan_sim_results/omni_ipc_core
```

这一步依次运行 C++ 测试、17 项 Python 检查、omni 的 6 个闭环场景，以及 diff 的 6 个场景作为回归对照。它不使用 ROS master，也不连接硬件。

重点查看：

```text
~/neupan_sim_results/omni_ipc_core/ctest.log
~/neupan_sim_results/omni_ipc_core/python_tests.log
~/neupan_sim_results/omni_ipc_core/core_omni/summary.json
~/neupan_sim_results/omni_ipc_core/core_diff/summary.json
```

预期 C++ 为 `100% tests passed`；两种模型的场景摘要均为 `passed: 6, total: 6`。omni 的 `max_yaw_drift` 应为 0，最大速度不超过 0.2 m/s，两个内部控制量的最大变化率分别不超过 0.3 m/s² 和 3.0 rad/s（允许数值舍入误差）。结果目录已存在时，闭环脚本会生成带时间后缀的新子目录，并在终端打印实际位置。

## 4. RViz 可视化仿真

重新开终端，加载本次环境：

```bash
source ~/neupan_omni_build/activate.bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  config_file:="$(rospack find neupan_cpp_ros1)/config/fw_mini_omni.yaml" \
  output:="$HOME/neupan_sim_results/omni_ipc_rviz"
```

RViz 打开后使用 **2D Nav Goal** 点选终点。当前显示配置的目标话题为 `/neupan_sim/goal`，Fixed Frame 为 `neupan_sim/map`。用 **2D Pose Estimate** 可重置虚拟小车位置和初始朝向。

omni 版本会改变平移方向，车身朝向保持不变。可观察矩形车体、方向箭头、A* 参考路径、C++ 局部预测轨迹和实际轨迹。

另开终端可检查：

```bash
source ~/neupan_omni_build/activate.bash
rostopic echo /neupan_sim/cmd_vel
```

应看到 `linear.x/linear.y` 根据移动方向变化，`angular.z=0`。点击终点后，如果需要横向或斜向平移，`linear.y` 应有非零值。车身当前朝向不同会改变车体坐标下的两个速度分量。

让仿真自动走向预设终点，可在上述 launch 命令中增加 `auto_goal:=true`。`Ctrl+C` 正常退出后，结果保存在输出目录下的 `summary.json`、`trajectory.csv` 等文件中。输出目录重复时会自动添加时间后缀。

**必须显式传入 `fw_mini_omni.yaml`。** `sim_complete.launch` 默认仍使用归档的 diff 配置。切回 diff 对照时将文件名改为 `fw_mini_balanced.yaml`，并换一个结果目录即可。

## 5. ROS1 自动闭环与断流测试（可选）

```bash
source ~/neupan_omni_build/activate.bash
cd ~/work/neupan_omni_noetic_bundle_20261027/neupan_cpp_ros1
bash tools/validate_ipc.sh ros ~/neupan_sim_results/omni_ipc_ros
```

它自动创建独立的本地 ROS master，测试 omni 横移、初始朝向旋转 90°、diff 直行、激光/TF 断流停车，再使用同一个 `sim_complete.launch` 运行无窗口走廊场景。此项不需要人工点选终点；完整过程约数分钟，具体取决于工控机负载。

查看 `~/neupan_sim_results/omni_ipc_ros/ros1/summary.json`。预期四项用例均 `passed: true`，走廊为零碰撞、`arrived: true`。重复运行时也可能生成带时间后缀的子目录。

## 6. 对照已有结果

此前验证结果保存在本包的 `neupan_cpp_ros1/test/results/`；详细说明见 `neupan_cpp_ros1/OMNI_VALIDATION_CN.md`。

核心和 ROS1 仿真结果存在时间离散、调度和积分差异，不要求每个轨迹点完全一样。复测应检查全部用例通过、零碰撞、到达、固定朝向、速度/方向变化率限制，以及断流停车。

本包的 `fw_mini_bridge.py` 保留 diff 档案行为；本次 RViz/核心复测不启动它。档位 7 的 CAN 实车桥接尚未包含在此次模型移植中。
