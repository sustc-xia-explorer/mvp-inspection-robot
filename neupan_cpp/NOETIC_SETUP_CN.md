# Ubuntu 20.04 / ROS Noetic：从复制源码重新部署 NeuPAN C++

本说明对应本目录内已经修改的源码。阶段一使用 FW-mini 模式二的 diff 等效控制；需先验证 CAN 能连续执行纵向速度和角速度组合。模式三横移及三自由度控制不在当前 C++ 核心支持范围内。

## 1. 使用这一份源码，建立独立环境

已经提供 `neupan_noetic_bundle.tar.gz`。包中有修改后的 `neupan_cpp`、OSQP、osqp-eigen、QDLDL 源码，不需要复用旧电脑的 build/devel，也不需要重新联网克隆这些仓库。

复制压缩包到新工控机的 `~/work` 后执行：

```bash
mkdir -p ~/work
tar -xzf ~/work/neupan_noetic_bundle.tar.gz -C ~/work
cd ~/work/neupan_noetic_bundle/neupan_cpp
```

如果直接使用当前完整 `work` 目录，则源码入口是：

```bash
cd ~/work/mvp_ws/src/neupan_cpp
```

这两种入口任选一种。后续命令都在所选源码根目录执行。安装脚本会建立：

```text
~/.local/neupan-noetic/       OSQP、OsqpEigen、libneupan 的独立安装位置
~/neupan_noetic_build/       全新构建目录、activate.bash
~/neupan_noetic_ws/          只含 ROS1 接口包的 catkin 工作空间
```

它不把 libneupan 混入 mvp_ws，不覆盖 `/usr/local`，也不修改 FAST-LIO/PCL 的安装。不要同时 source Humble 或旧的混合工作空间。

## 2. 检查 Noetic，安装基础依赖

要求新工控机已经安装 Ubuntu 20.04 和 ROS Noetic。本步骤不包含安装操作系统和 ROS 发行版。

```bash
source /opt/ros/noetic/setup.bash
rosversion -d

sudo apt update
sudo apt install -y build-essential cmake git curl ca-certificates \
  libeigen3-dev libyaml-cpp-dev libgtest-dev python3-rospkg \
  ros-noetic-catkin ros-noetic-roscpp ros-noetic-rospy ros-noetic-roslib \
  ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-geometry-msgs \
  ros-noetic-visualization-msgs ros-noetic-std-msgs \
  ros-noetic-diagnostic-msgs ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros \
  ros-noetic-pointcloud-to-laserscan
```

`rosversion -d` 应输出 `noetic`。保留 Ubuntu 20.04 的系统 Eigen 3.3；此次兼容补丁支持它。

## 3. 准备 CMake 3.x

```bash
cmake --version
```

如果已经是 CMake 3.18～3.x，直接跳到第 4 步；脚本会使用它。若仍是系统 3.16.3，可以额外下载 3.22.6 到用户目录：

```bash
mkdir -p ~/.local/opt
curl -L --fail \
  https://github.com/Kitware/CMake/releases/download/v3.22.6/cmake-3.22.6-linux-x86_64.tar.gz \
  -o /tmp/cmake-3.22.6-linux-x86_64.tar.gz
tar -xzf /tmp/cmake-3.22.6-linux-x86_64.tar.gz -C ~/.local/opt
export NEUPAN_CMAKE="$HOME/.local/opt/cmake-3.22.6-linux-x86_64/bin/cmake"
"$NEUPAN_CMAKE" --version
```

此下载文件面向 x86_64，适用于这里的 Intel 工控机。系统 `/usr/bin/cmake` 可以保留原版。新终端中继续构建时，需要重新设置 `NEUPAN_CMAKE`，或明确使用已经满足版本要求的 cmake。

## 4. 源码具体改了什么

核心共修改 4 个文件：

1. `libneupan/src/mlp.cpp`：bias、LayerNorm gamma/beta 的三处展开替换为：

   ```cpp
   Eigen::Map<const Vec>(m.data(), m.size()).cast<float>()
   ```

2. `libneupan/src/dune.cpp`：将 `mh.reshaped()` 替换为：

   ```cpp
   Eigen::Map<const Vec>(mh.data(), mh.size())
   ```

3. `libneupan/include/neupan/nrmp.hpp`：移动函数声明调整为：

   ```cpp
   NRMP(NRMP&&);
   NRMP& operator=(NRMP&&);
   ```

4. `libneupan/src/nrmp.cpp`：对应定义同步调整：

   ```cpp
   NRMP::NRMP(NRMP&&) = default;
   NRMP& NRMP::operator=(NRMP&&) = default;
   ```

当前 `Mat` 是列主序 `Eigen::MatrixXd`，这些 Map 保持原展开顺序。`libneupan/CMakeLists.txt` 已要求 Eigen >=3.3；保留此要求。

ROS1 接口的完整文件已提供，无需再逐条手工替换：

- `neupan_cpp_ros1/src/neupan_node.cpp`：roscpp、ROS1 消息、NodeHandle 私有参数、发布订阅、ROS 时间/Timer、TF 查询和日志。
- `neupan_cpp_ros1/CMakeLists.txt`：catkin 构建，通过 `neupan::neupan` 链接独立核心库。
- `neupan_cpp_ros1/package.xml`：ROS1 依赖声明。
- `neupan_cpp_ros1/launch/neupan_cpp_ros1.launch`：参数和正确包名；默认发布 `/neupan_cmd_vel`。

接口会在首次有效扫描之前、扫描超时、机器人 TF 超时和求解失败时发布零速度；求解失败宽限默认 0，可配置。扫描使用消息中的实际 frame 和时间戳，通过完整 TF 变换后取平面坐标。路径/目标必须明确使用与 `map_frame` 相同的 frame。

源码目录中的 `astar_global_node.cpp`、旧 ROS2 `.launch.py` 和 `test/*.py` 仅留作参考，不参与当前 ROS1 构建。不要运行旧的 ROS2 启动命令。原始 `config/planner.yaml` 也保留作参考，新的启动文件使用下述配置。

## 5. 一次构建依赖、核心和 ROS1 包

回到第 1 步选定的源码根目录：

```bash
bash tools/build_noetic.sh all
```

默认并行数为 2，适合先在工控机上降低内存压力。需要时可指定：

```bash
NEUPAN_JOBS=4 bash tools/build_noetic.sh all
```

脚本顺序为 OSQP → osqp-eigen → libneupan → 核心测试 → 安装核心 → catkin 构建接口。只验证核心可用 `bash tools/build_noetic.sh core`。不会要求安装 PyTorch、cvxpy、LibTorch 或 CUDA。

成功应看到 `100% tests passed` 以及 `Built target neupan_node`。然后：

```bash
source ~/neupan_noetic_build/activate.bash
rospack find neupan_cpp_ros1
```

依赖源码路径、构建路径和安装位置可分别通过 `NEUPAN_DEPS_SOURCE_ROOT`、`NEUPAN_BUILD_ROOT`、`NEUPAN_INSTALL_PREFIX`、`NEUPAN_ROS_WS` 覆盖。若把源码移动到另一个路径后重建，选择新的构建目录，避免复用记录旧路径的 CMakeCache。

## 6. 先做软件接口自检

终端 A：

```bash
source ~/neupan_noetic_build/activate.bash
roscore
```

终端 B：

```bash
source ~/neupan_noetic_build/activate.bash
rosrun neupan_cpp_ros1 smoke_test.py
```

自检会启动独立命名空间中的规划器，发布合成 TF、无障碍 LaserScan 和参考路径，用虚拟差速车执行输出，并检查：首次扫描前零速度、虚拟车前进、停止扫描后零速度。通过时输出 `PASS`。它不发布 `/cmd_vel` 或 `/ctrl_cmd`，也不启动 CAN 驱动。

这里使用 `config/smoke.yaml` 和随包的 `models/diff_default.bin`，几何为原模型的 1.6×2.0 m。该自检验证软件接口和基本闭环，不代表 FW-mini 地图避障或实车测试通过。

在实车前继续运行 [场景仿真与 Python/C++ 对照](SIMULATION_VALIDATION_CN.md)。其中提供使用实际 FW-mini 模型的 7 场景测试、CSV/轨迹图输出，以及已发现的转弯和走廊未通过项。

## 7. 在开发电脑训练 FW-mini DUNE，再导出

使用你已配置好的 Python NeuPAN 环境。先核实实际车体外轮廓：默认 0.495×0.360 m 只来自底盘手册；突出设备必须计入。`base_link` 暂按轮廓中心定义。

同时修改以下两个配置的尺寸：

- `neupan_cpp_ros1/config/fw_mini_dune_train.yaml`
- `neupan_cpp_ros1/config/fw_mini.yaml`

这里 `wheelbase: 0.0` 对应几何参考点约定，不能把物理轴距 0.35 m 直接填进来；本实现的该参数会平移 footprint。若 base_link 不在几何中心，需要统一训练和规划的参考点约定。

在开发电脑、这份 C++ 源码根目录执行：

```bash
source ~/NeuPAN/.venv/bin/activate
python tools/train_fw_mini.py
```

训练脚本使用 `direct_train: true` 初始化，再显式训练一次；模型输出到 `tools/model/fw_mini/`，重复运行可能产生带后缀目录，以训练完成时打印的实际位置为准。环境中的 `neupan` 必须能正常导入。

按实际权重路径导出（尺寸示例须与训练完全一致）：

```bash
python libneupan/tools/export_dune_weights.py \
  tools/model/fw_mini/model_5000.pth \
  neupan_cpp_ros1/models/fw_mini.bin 0.495 0.360 0.0
```

将 `fw_mini.bin` 复制到工控机相同模型目录。修改默认模型的元数据不能代替训练；只有训练几何、导出几何和规划几何一致时距离估计才有意义。

## 8. 接入 Livox / FAST-LIO

先单独确认雷达驱动、FAST-LIO 在新工控机能运行，已有地图路径是 `work/ws_livox/src/FAST_LIO/PCD/scans.pcd`。复制源码不保证新电脑已经安装 Livox SDK、PCL 和 ROS 依赖。

本 ROS1 节点接收 `sensor_msgs/LaserScan`。FAST-LIO 的 `/cloud_registered_body` 是 PointCloud2。先明确本工程的坐标系：

| 名称 | 含义 | 当前来源 |
| --- | --- | --- |
| camera_init | FAST-LIO 启动时建立的局部世界坐标系 | FAST-LIO |
| body | 雷达内置 IMU 坐标系，随雷达安装方向倾斜 | FAST-LIO |
| base_link | 本方案约定的底盘几何中心，X 向前、Y 向左、Z 向上 | 需要按实际安装外参补充 |

所以，FAST-LIO 运行后只有 `camera_init -> body` 是正常的；`body` 不是已经标定好的底盘坐标系。

下面命令先加载 NeuPAN 工作空间环境，再启动点云转 LaserScan 节点。`cloud_topic` 选输入点云，`target_frame` 选变换和高度筛选所用的坐标系；它不会自动创建 base_link 或完成外参标定：

```bash
source ~/neupan_noetic_build/activate.bash
roslaunch neupan_cpp_ros1 livox_scan.launch \
  cloud_topic:=/cloud_registered_body target_frame:=base_link
```

节点名称和参数参考 [pointcloud_to_laserscan 上游 ROS1 示例](https://github.com/ros-perception/pointcloud_to_laserscan/blob/indigo-devel/launch/sample_node.launch)。

如果现在只有 `camera_init -> body`，先测量雷达 IMU 与底盘中心之间的刚性变换，并补充 `body -> base_link` 静态 TF。当前底盘驱动的 `tfUsed: false` 可保留；不要让其他 TF 发布者同时给 base_link 指定另一个父坐标系。

ROS1 命令格式如下，字母是需要替换的测量/标定值，不可直接执行：


### 待标定

```text
rosrun tf2_ros static_transform_publisher x y z qx qy qz qw body base_link
```

这里平移是 **base_link 原点在 body 坐标系中的坐标**，四元数是 base_link 相对 body 的姿态。若测得的是 IMU 在底盘系中的位姿，需对完整刚性变换求逆；雷达倾斜时不能仅把平移取负。没有实测依据时，不要全部填 0 或把雷达原点直接当底盘中心。

安装倾斜时，仅将 `target_frame:=body` 会在 IMU 坐标系中按 Z 裁剪，得到不适合底盘平面导航的截面。先用外参把点云变换到正确底盘系，再做二维投影。

前提是已经完成安装外参标定，目标 frame 的 X 向前、Y 向左、Z 向上，并且其 Z 方向适合地面高度筛选。`min_height/max_height` 是相对该目标 frame 的高度，示例值必须根据实际坐标原点和障碍高度调整。需要在投影前去掉车体回波；本 launch 只负责坐标变换后的高度/距离筛选和二维投影，不包含车体裁剪或通用地面分割。

FAST-LIO 配置中的 `extrinsic_T/R` 是雷达与 IMU 之间的关系，不能代替雷达到底盘的安装外参。不要在倾斜雷达的原始坐标系里直接按 Z 裁剪。

```bash
rostopic type /scan
rostopic hz /scan
rosrun tf tf_echo camera_init body
rosrun tf tf_echo camera_init base_link
```

类型应是 `sensor_msgs/LaserScan`。检查命令属于 ROS1 的 tf 包；先前写成 `rosrun tf2_ros tf2_echo ...` 的命令在此更正。

还需要确认世界坐标系的 XY 平面对应实际地面。这份 FAST-LIO 初始化代码估计重力方向，但并未由此自动把初始姿态旋转到重力对齐，不能仅凭 camera_init 这个名称认定其 Z 轴向上。如果 camera_init 已满足平面导航约定，启动 NeuPAN 时可传 `map_frame:=camera_init`；否则需先在它上方建立重力对齐的 map，使用 `map_frame:=map`。新增 map 只能有一个确定的发布者。若使用 NDT 固定地图定位，则统一为它实际发布的地图坐标系。不要用身份变换掩盖尚未标定的安装位置。

## 9. 启动 FW-mini 规划器，先观察输出

本步骤要求模型和 TF 已就绪，但不自动启动底盘驱动：

```bash
source ~/neupan_noetic_build/activate.bash
roslaunch neupan_cpp_ros1 fw_mini.launch \
  dune_checkpoint:="$(rospack find neupan_cpp_ros1)/models/fw_mini.bin"
```

仅在已确认 camera_init 的 XY 平面对应地面时，追加 `map_frame:=camera_init`；若已建立重力对齐的 map，则保留默认 `map_frame:=map`。

输入路径用 `/initial_path`（nav_msgs/Path，至少两个点）；也可发送 `/neupan_goal` 或 `/neupan_waypoints`。这些消息的 `header.frame_id` 必须与启动的 `map_frame` 一致。例如，在 `map` 中，从当前姿态规划到实际确认过的目标点：

```bash
rostopic pub -1 /neupan_goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
rostopic echo /neupan_cmd_vel
```

示例目标坐标只用于说明接口，运行前替换成当前地图中的有效目标。当前版本只有直线路径处理，不会仅凭目标点自动绕过墙体生成全局可行路径；复杂地图应提供经过全局规划的 `/initial_path`。

`fw_mini.yaml` 起步值为参考速度 0.15 m/s、最大纵向速度 0.2 m/s、最大角速度 0.4 rad/s。需要在底盘模式二 CAN 控制验证后再调整；这些参数不构成已验证的车辆动力学模型。

## 10. 底盘接入前的具体接口要求

当前 FW-mini 驱动订阅 `/ctrl_cmd`，消息是 `yhs_can_msgs/ctrl_cmd`。NeuPAN 的 `/neupan_cmd_vel` 是 Twist。需要速度转换节点，不能靠 remap 改变消息类型。

| 输入 Twist | 底盘控制消息 | 转换 |
| --- | --- | --- |
| linear.x | ctrl_cmd_x_linear | m/s，按当前驱动约定 |
| linear.y | ctrl_cmd_y_linear | 阶段一强制为 0 |
| angular.z | ctrl_cmd_z_angular | 当前驱动要求 °/s，乘 180/π |
| 模式/控制权配置 | ctrl_cmd_gear | 必须核实固件实际值 |

先核实遥控模式二对应的 CAN 控制模式/档位、正负方向、连续 v/ω 组合及轮组切换时间。遥控模式编号不能直接写入 gear。模式三能横移也不表示当前 C++ 已支持 omni。

原驱动每 10 ms 反复发送缓存命令，所以速度转换节点和底盘驱动都要有超时归零。驱动层可按下面方式补：

1. 在 CanControl 成员中增加 `ros::WallTime last_ctrl_cmd_time_; bool ctrl_cmd_seen_ = false;`。
2. `ctrl_cmdCallBack()` 在已有 `cmd_mutex_` 锁内更新缓存的同时写入 `last_ctrl_cmd_time_ = ros::WallTime::now(); ctrl_cmd_seen_ = true;`。
3. `timerCallBack()` 在同一锁内、发送控制帧之前，对 `MODE_CTRL_CMD` 检查首次命令和超时（起步可设 0.3 s）；不满足时把缓存中的 x/y/z 三个速度字段归零。gear 保留已核实的控制模式，停车/禁用档位行为另按固件协议实现。
4. 转换节点同样限制最大速度、检查非有限值，并在输入超时后持续输出零速度；驱动的独立超时负责覆盖转换节点退出的情况。

本次包内提供规划器部署和验证代码；未知的固件 gear 值、控制权切换和 CAN 接口没有硬编码成可发车指令。实车接入还需按这台替换后的工控机/底盘验证这些接口。

## 11. 验收与轻量化实验

核心回放测试会保留数值回归并打印耗时，默认不套用原开发机的 5 ms 性能门限。需要性能验收时，使用 `NEUPAN_REPLAY_MAX_MEAN_MS=5 ctest --test-dir ~/neupan_noetic_build/libneupan -R '^test_replay$' -V` 显式启用，超时仍会失败。均值略超 5 ms 不代表数值移植失败；实际控制周期的实时性应另行测量。

实车前可先运行 [RViz 点选终点的 FW-mini 交互仿真](RVIZ_SIMULATION_CN.md)。其中使用单独的 A* 全局路径模块、模拟扫描和虚拟底盘，入口为 `sim_complete.launch`；现有实车规划节点接收路径的接口保留。工控机已有仿真证据见 [结果分析](IPC_RESULTS_ANALYSIS_CN.md)。

按顺序验收：核心三个测试通过 → ROS1 自检通过 → 实际 TF/扫描/路径输入正确 → 低速 CAN 指令及超时停车正确 → 实车闭环。

轻量化比较时，在同一 i5 上给 Python/C++ 相同点云投影、模型、路径、预测步数、点数和迭代数，记录完整规划周期的平均/P95/P99 耗时、CPU、内存、跟踪误差、最小间距和成功率。先比较实现差异，再单独比较减少点数/步数/迭代数的影响。兼容性补丁通过不等于轻量化效果或实车安全性已经得到验证。

## 验证记录

实际验证结果另见 `VALIDATION_CN.md`。当前开发电脑是 Ubuntu 22.04 / Humble，不应把这里的核心验证写成新工控机上的 Noetic 实测。
