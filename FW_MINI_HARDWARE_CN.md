# FW-mini 驱动接入与初次实机测试

**2026-10-26 归档状态：本文是已冻结 diff 版本的历史接入与排查记录。** 厂商已确认本车不能按原设想锁后轮使用 diff，后续采用全向底盘路线。下文保留旧启动/使能步骤供复核，不作为新的实车运动操作流程。当前代码没有实现 omni，见 [归档说明](DIFF_ARCHIVE_CN.md) 和 [全向适配评估](OMNI_ADAPTATION_CN.md)。

新增入口为 `neupan_cpp_ros1/launch/fw_mini_hardware.launch`。规划器仍是 C++，薄接口 `fw_mini_bridge.py` 负责约 50 Hz 的限幅、单位换算、使能、超时与日志。它连接当前工作区实际存在的 ROS1 `yhs_can_msgs/ctrl_cmd`，不启动电机使能或 CAN 初始化。

```text
FAST-LIO2 + body→base_link 外参 + /scan + 参考路径
    → C++ neupan_node → /neupan_cmd_vel (Twist，m/s、rad/s)
    → fw_mini_bridge → /ctrl_cmd (yhs_can_msgs/ctrl_cmd)
    → 现有 yhs_can_control → SocketCAN → FW-mini

桥接已下发指令 → /fw_mini/applied_cmd_vel (TwistStamped)
    → 规划器首拍边界与记录；不是编码器测量速度
```

## 1. 消息与协议约定

| Twist / 参数 | 驱动字段 | 转换 |
| --- | --- | --- |
| `linear.x` | `ctrl_cmd_x_linear` | m/s，按配置限幅 |
| `angular.z` | `ctrl_cmd_z_angular` | rad/s → deg/s |
| diff 模型 | `ctrl_cmd_y_linear` | 固定 0，拒绝非零横向输入 |
| 必填 `ctrl_gear` | `ctrl_cmd_gear` | 原样传入 4 bit 字段，范围 0…15；具体合法值按固件确认 |

当前驱动 `sendCtrlCommand()` 对 x/y 乘 1000、对角速度乘 100，不执行弧度到角度换算。因此桥接按角速度字段为 deg/s、CAN 为 0.01 deg/s 的协议解释，换算放在桥接里。如果换过底盘固件或驱动版本，需要先核对协议和架空测试的角速度反馈。**遥控模式二/三不能当作 `ctrl_cmd_gear` 值；本次没有猜测或默认指定实车挡位。**

`max_speed=[0.20,0.40]`、`max_acce=[0.30,0.60]` 直接读取 balanced YAML，分别使用 m/s、rad/s、m/s²、rad/s²。支持的是前进/后退与转向组合的 diff 接口，尚未实现三自由度全向规划。

## 2. 更新源码、驱动补丁并构建

将更新包解压到原源码上层目录，例如 `~/work/neupan_noetic_bundle`，包内顶层为 `neupan_cpp/`。先备份旧源码，再解压。保留既有 `deps/`，无需升级系统 Eigen、PCL 或 ROS。

以下命令从新源码根目录执行。默认驱动位于 `~/work/mvp_ws/src/FW-mini-ros`；若位置不同替换路径。

```bash
python3 tools/apply_fw_watchdog.py \
  --driver-root ~/work/mvp_ws/src/FW-mini-ros
```

脚本核对两个源文件的 SHA256；完全匹配旧版本才备份并应用补丁，已应用则直接提示。若源文件不同，停止修改，需对照 `hardware/fw_mini_watchdog.patch` 合并。当前开发机相邻驱动目录已经应用，工控机仍需执行。

补丁令整体控制模式在未收到指令、指令含 NaN/Inf、超过 0.3 s 未收到 `/ctrl_cmd` 时清零三个速度分量；发送定时器使用 WallTimer，不依赖 `/clock`。其他独立轮控模式未纳入本接口验收。

```bash
source /opt/ros/noetic/setup.bash
cd ~/work/mvp_ws
catkin_make --only-pkg-with-deps yhs_can_control -DCMAKE_BUILD_TYPE=Release

cd ~/work/neupan_noetic_bundle/neupan_cpp
NEUPAN_DRIVER_WS="$HOME/work/mvp_ws" \
  bash tools/build_noetic.sh all

source ~/neupan_noetic_build/activate.bash
python3 -c 'from yhs_can_msgs.msg import ctrl_cmd; print("FW-mini ROS1 messages OK")'
```

`NEUPAN_DRIVER_WS` 让生成的激活脚本带上驱动工作空间，避免桥接运行时无法导入自定义消息。若依赖源码不在原脚本默认位置，继续使用此前有效的 `NEUPAN_DEPS_SOURCE_ROOT`。先按 [变化率复测说明](ACCELERATION_VALIDATION_CN.md) 运行核心回归和 RViz，再接实车。

## 3. 定位、激光和路径输入

### 3.1 当前工作空间与启动顺序

2026-10-24 按现有 `~/work/ws_livox` 中的实际 launch、配置和消息接口整理。沿用已经验证的雷达与 FAST-LIO2 代码，本节不要求重新下载或升级；本次整理没有重新连接雷达运行。

| 用途 | 当前文件 / ROS 包名 |
| --- | --- |
| Livox SDK2 源码 | `~/work/ws_livox/src/Livox-SDK2`，由驱动调用，不另起 ROS 节点 |
| MID360S 驱动入口 | `~/work/ws_livox/src/livox_ros_driver2/launch_ROS1/msg_MID360s.launch`，包名 `livox_ros_driver2` |
| 雷达网络配置 | `~/work/ws_livox/src/livox_ros_driver2/config/MID360s_config.json` |
| FAST-LIO2 入口 | `~/work/ws_livox/src/FAST_LIO/launch/mapping_mid360.launch`，**包名是 `fast_lio`** |
| FAST-LIO2 参数 | `~/work/ws_livox/src/FAST_LIO/config/mid360.yaml` |
| 点云转二维激光 | 本项目 `neupan_cpp_ros1/launch/livox_scan.launch` |

启动顺序：**ROS master → MID360S 驱动 → FAST-LIO2 → 安装外参 TF → 点云转 `/scan` → 底盘驱动 → 第 4 节 NeuPAN 与桥接 → 路径/目标 → 手动使能。** 雷达及点云检查阶段不需要启动底盘驱动。

下面每个“终端”均表示新开一个终端并保持其节点运行。雷达和 FAST-LIO2 终端加载 `ws_livox`；NeuPAN 终端加载自己的 `activate.bash`，其中应已包含第 2 节配置的 FW-mini 驱动工作空间。无需让所有工作空间都 source 到同一个终端；各节点使用同一个 ROS master 即可。不要在这些 Noetic 终端中加载 ROS2 Humble。

以下路径都用 `~`，在工控机上对应工控机当前用户目录。若工作空间曾从另一台电脑或另一用户名下复制，确认 `rospack find` 指向本机源码；旧 `build/devel` 缓存可能仍含原路径，不能仅凭目录存在就认定可直接使用。

### 3.2 启动前核对网络与环境

当前本地 `MID360s_config.json` 记录的工控机接收地址 `host_ip` 是 **192.168.1.125**，雷达地址 `lidar_configs[0].ip` 是 **192.168.1.188**。这是现有配置值，需与实车接线网口及雷达实际地址一致；已经通信正常时沿用现场配置。

```bash
ip -br -4 addr
ping -c 4 192.168.1.188

source /opt/ros/noetic/setup.bash
source ~/work/ws_livox/devel/setup.bash
rospack find livox_ros_driver2
rospack find fast_lio
```

两个 ROS 包应分别解析到本机 `~/work/ws_livox/src/livox_ros_driver2` 和 `~/work/ws_livox/src/FAST_LIO`。网络配置文件以该解析到的驱动目录为准，避免改了一份 JSON 却运行另一份代码。Ping 只检查基本连通性，数据是否正常还需看下面的话题。

**终端 0：ROS master。** 已有同一个 master 在运行时沿用，不重复启动。

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 3.3 启动 MID360S 雷达驱动

**终端 1：**

```bash
source /opt/ros/noetic/setup.bash
source ~/work/ws_livox/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch \
  xfer_format:=1 multi_topic:=0 publish_freq:=10.0
```

必须使用当前 MID360S 对应的入口。`xfer_format:=1` 发布 `livox_ros_driver2/CustomMsg`，供这份 FAST-LIO2 读取逐点时间；`multi_topic:=0` 保持 `/livox/lidar`、`/livox/imu` 与现有 YAML 一致。驱动的 `rviz_MID360s.launch` 默认 `xfer_format=0`，直接替换会变成 PointCloud2，不能直接匹配当前 FAST-LIO2 的 Livox 订阅回调。

在另一个检查终端加载相同 `ws_livox` 环境后执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/work/ws_livox/devel/setup.bash
rostopic type /livox/lidar
rostopic type /livox/imu
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

`rostopic hz` 持续统计，用 Ctrl+C 结束当前检查后再运行下一项。类型应分别为 `livox_ros_driver2/CustomMsg` 和 `sensor_msgs/Imu`；点云目标发布频率为 10 Hz，IMU 应持续更新。不要为了单独看原始点云再启动第二份 Livox 驱动；后续在 FAST-LIO2 的 RViz 中查看标准 PointCloud2 输出。

### 3.4 启动 FAST-LIO2

**终端 2：** 启动时保持车辆与雷达静止，等待 IMU 初始化及点云/位姿正常输出后再移动。

```bash
source /opt/ros/noetic/setup.bash
source ~/work/ws_livox/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch rviz:=true
```

`mapping_mid360.launch` 只启动 FAST-LIO2 与可选 RViz，不会自动启动上一节的雷达驱动。需要减少工控机可视化负载时可用 `rviz:=false`；正式耗时对比时保持各组 RViz 开关一致。

当前 `mid360.yaml` 已配置：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
preprocess:
  lidar_type: 1
publish:
  scan_publish_en: true
  scan_bodyframe_pub_en: true
```

关键输出与检查：

| 输出 | 类型 / 含义 |
| --- | --- |
| `/cloud_registered` | `sensor_msgs/PointCloud2`，`camera_init` 世界系点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2`，`body` 即 IMU 系点云，作为本项目二维转换输入 |
| `/Odometry` | `nav_msgs/Odometry`，注意大写 O，父 frame 为 `camera_init`、子 frame 为 `body` |
| TF `camera_init → body` | FAST-LIO2 估计的动态位姿 |

```bash
rostopic type /cloud_registered_body
rostopic echo -n 1 /cloud_registered_body/header
rostopic hz /cloud_registered_body
rostopic echo -n 1 /Odometry/header
rosrun tf tf_echo camera_init body
```

RViz 的 Fixed Frame 选 `camera_init`，添加 `/cloud_registered` 查看建图。只有 `camera_init` 和 `body` 是正常的，FAST-LIO2 不会自动创建底盘 `base_link`。

现有配置同时启用了 `pcd_save_en: true`、`interval: -1`，会累积待保存点云，正常退出时写入编译时源码目录下的 `PCD/scans.pcd`。当前源码已有 `~/work/ws_livox/src/FAST_LIO/PCD/scans.pcd`；需要保留旧地图时先备份，正常退出可能覆盖同名文件。长时间纯导航而无需保存点云时，可在 `mid360.yaml` 中关闭 `pcd_save_en` 后重启 FAST-LIO2，避免持续缓存。本启动流程是在线建图/里程计，不会自动加载这张旧 PCD 做固定地图重定位。

### 3.5 发布雷达 IMU 到底盘的安装外参

**终端 3：** 先加载 `source /opt/ros/noetic/setup.bash`，再运行已经标定好的静态 TF 发布命令。格式如下，字母必须替换成实测值，不能直接照抄执行：

```text
rosrun tf2_ros static_transform_publisher x y z qx qy qz qw body base_link
```

平移是 **base_link 原点在 body 坐标系中的坐标**，四元数表示 base_link 相对 body 的姿态。若测得的是 IMU 在底盘系中的位姿，需要对完整刚体变换求逆；倾斜安装时不能只把平移取负，也不能全部填 0。`base_link` 应与模型的车体参考点一致，x 前、y 左、z 上。FAST-LIO2 的 `mapping/extrinsic_T/R` 是雷达与 IMU 之间的外参，不代替这里的安装外参。

```bash
rosrun tf tf_echo body base_link
rosrun tf tf_echo camera_init base_link
```

已有 URDF 或其他节点发布同一外参时沿用，避免重复发布。当前底盘配置 `tfUsed: false` 可保持，避免同时出现 `odom → base_link` 和 `body → base_link` 两个父 frame。

还需检查 `camera_init` 的 XY 是否对应实际地面。当前 FAST-LIO2 会估计重力，但不能仅凭 frame 名称认定世界系 Z 已对齐重力。雷达倾斜安装时，若世界系仍不适合平面导航，应先建立经标定的地面对齐 `map`，后续使用 `map_frame:=map`；本节及第 4 节的 `camera_init` 命令以它已满足平面导航约定为前提。具体外参说明见 [NOETIC_SETUP_CN.md 第 8 节](NOETIC_SETUP_CN.md#8-接入-livox--fast-lio)。

#### 3.5.1 当前车头立装照片对应的估算初值

**本节 90° 数值是照片估算的历史初值。已取得 1000 样本均值后，使用第 3.5.2 节的修正值替换两条静态 TF，不与本节命令同时运行。**

2026-10-24 根据提供的安装照片补充：雷达贴在车头中间，光学罩朝车前，航空接头朝上。结合 [MID360S 官方手册第 19、22 页](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf) 的坐标轴与 IMU 位置，初步推断为 **body 的 X 朝车下、Y 朝车左、Z 朝车前**，即 body 相对底盘绕 Y 轴约 +90°。这是照片推断，不能替代安装角度测量。

为使本组数值可复现，明确约定：`base_link` 的 XY 原点在底盘轮廓中心，Z 原点取地面，x 前、y 左、z 上。若你的已有 base_link 高度不是地面，需修改高度差，不能混用本组平移。

| 估算项 | 初值与依据 |
| --- | --- |
| 底盘长、宽、高 | FW-mini V1.0.3 手册表 2-1：0.495 × 0.360 × 0.320 m |
| 车头安装面到中心 | 暂取半车长 0.2475 m；实际前板位置、胶垫/支架厚度待量 |
| 光学原点离地高度 | 先假设约 0.25 m；这是照片估算，不是底盘手册给定尺寸 |
| IMU 相对光学原点 | 雷达手册给出 `[0.011, 0.02329, -0.04412]` m，当前 FAST-LIO2 的内外参也对应此位置差 |
| body 原点在 base_link 中的位置 | 上述假设换算并取整后，约 `[0.25, 0.023, 0.24]` m |
| body 相对 base_link 姿态 | roll≈0、pitch≈+90°、yaw≈0 |

外壳居中并不意味着 IMU 芯片恰在车体中线上，约 0.023 m 的 Y 偏移来自芯片位置。本组高度和平移均可能有数厘米误差，保留的小数用于计算一致性，不表示已达到毫米级标定精度。

按本工程所需的 **父 body、子 base_link** 方向，对完整变换求逆后，终端 3 可先使用下列估算值检查点云和 TF：

```bash
source /opt/ros/noetic/setup.bash
rosrun tf2_ros static_transform_publisher \
  0.24 -0.023 -0.25 0 -0.70710678 0 0.70710678 body base_link
```

这里的前三项已经是底盘原点在倾斜 body 系中的坐标，不能误写为 `-0.25 0 -0.24`；后四项顺序为 `qx qy qz qw`。不要与前面已有的同一条外参同时发布。

**立装时还需调平导航世界系。** 当前 `IMU_Processing.hpp` 初始化估计重力，但未自动将初始姿态旋转到地面水平；若 FAST-LIO2 从初始单位姿态、零位置启动，`camera_init` 仍沿初始 IMU 轴方向。仅发布 body→base_link 不能改变世界系 XY 平面。在这一前提下，另开终端 3B，使用下面的估算静态变换定义本次启动的地面对齐 `map`：




两条静态变换互为逆，配合初始 `camera_init→body` 为单位变换时，使初始 `map→base_link` 约为零位姿。之后 FAST-LIO2 的动态 TF 正常表达车辆运动。这个 map 只是本次启动定义的导航世界系，不是加载旧 PCD 的固定地图定位；已有其他 map 发布者、不同初始姿态或定位模块时，应使用实际变换，不能重复发布本条。

使用这套立装初值时，第 4 节传 `map_frame:=map`，路径和 RViz Fixed Frame 也改为 `map`。车辆保持静止，先验证：




静止加速度应主要落在 body 的负 X 方向；若主要为其他轴或正 X，照片推断的安装方向不适用，应先核对轴向。RViz 中 base_link 的 X 应向车前、Z 应向上，地面点应接近地面平面。注意两条互为逆的估算静态 TF 本身就会令初始 `map→base_link` 近于零；其 roll/pitch 接近零只能说明链路自洽，不能独立证明地面已经调平。需要结合重力在 base_link 中的方向和地面点验证。车辆向前小幅移动时，map 中的位移方向应与底盘 X 一致。

本组参数供静止点云对齐和初步验证，先尺量光学原点高度、安装面距车体中心距离并核对旋转，再用于落地运动。雷达罩突出车头，最终避障 footprint 还应包含该突出部分，不能仅以原底盘 0.495 m 外轮廓判断碰撞。

#### 3.5.2 根据 1000 个静止 IMU 样本修正旋转

2026-10-24 实机返回 1000 个样本，平均加速度为：

```text
[-0.9820701718330384, -0.02337134000658989, 0.1737365095615387]
```

按车辆静止、地面近水平的采集前提，模长约为 0.997593 g。当前驱动原样发布 Livox 加速度，FAST-LIO2 会按重力尺度归一化。原 +90° 旋转将其变为 `[0.173737, -0.023371, 0.982070]`，相对竖直仍偏约 **10.12°**。截图中 Fixed Frame 已切到 map，但调平应结合重力和地面点判断，不能只凭观察视角判断。

修正方法是在原照片姿态附近选择最小附加旋转，使归一化平均加速度映射到 base_link 的 +Z。重力不能确定航向，也不能测量平移；航向仍以照片朝向为先验，IMU 在底盘系中的位置仍暂用 `[0.25, 0.023, 0.24]` m。旋转修正后，对完整刚体变换重新求逆，不能保留旧的逆变换平移。

先用 Ctrl+C 停止第 3.5.1 节两条旧静态 TF 发布进程，再分别在两个终端启动以下命令。每个 child frame 只保留一个发布者。

**终端 3，body→base_link：**

```bash
source /opt/ros/noetic/setup.bash
rosrun tf2_ros static_transform_publisher \
  0.193265 -0.017885 -0.288024 \
  0.00831536 -0.64253640 0.00831536 0.76616492 \
  body base_link
```

**终端 3B，map→camera_init：**

```bash
source /opt/ros/noetic/setup.bash
rosrun tf2_ros static_transform_publisher \
  0.25 0.023 0.24 \
  -0.00831536 0.64253640 -0.00831536 0.76616492 \
  map camera_init
```

第二条仍沿用 FAST-LIO2 初始 `camera_init→body` 为零位置、单位姿态的约定。若系统采用其他初始化、已有定位模块或 map 发布者，不能直接套用第二条，需结合实际初始位姿计算。后续继续使用 `map_frame:=map`，RViz Fixed Frame 也选 map；`mid360.yaml` 中的雷达—IMU 内外参保持原值，本次只修正安装 TF。

理论检查：修正后的旋转将给定均值变为约 `[0, 0, 0.997593]` g，两条刚体变换相乘为单位变换；这些是数值一致性检查，不是实车标定验收。计算与假设保存在 [安装 TF 修正记录](docs/results/fw_mini_imu_mount_20261024.json)。

替换后，在保持车辆静止的情况下，检查地面与 map 的 XY 网格是否平行；在 base_link 下查看点云，地面高度应接近 0，仍需用实际高度修正整体偏移。原始 `/livox/imu` 数值不会因为发布 TF 而改变，验证重力方向时需要把它旋转到 base_link。朝向则需要用实车前方已知目标或人工推车方向进一步确认，不能仅靠加速度确认。

#### 3.5.3 为什么 RViz 仍可能看到 X 轴朝地

2026-10-24 再次采样的 1000 点均值为 `[-0.9819647031, -0.0233698752, 0.1737284529]`，与前次均值的方向差约 0.0006°。按第 3.5.2 节旋转换算，约得到 `[0.00001044, -0.00000102, 0.99748805]` g。这说明两次平均方向一致，无需因为原始数据仍主要在负 X 方向就再次更换四元数；该差值不代表真实标定精度。

立装雷达的 body 与底盘 base_link 是不同坐标系。静态 TF 描述它们之间的关系，不会重新定义原始 body 轴：

| frame | X（红） | Y（绿） | Z（蓝） |
| --- | --- | --- | --- |
| body | 本次安装下大致朝地，约有 10° 倾斜 | 大致朝车左 | 大致朝车前 |
| base_link | 应朝车前、近似平行地面 | 应朝车左、近似平行地面 | 应朝上 |
| map | 水平导航平面的一条轴 | 水平导航平面的另一条轴 | 应朝上 |

当前 FAST-LIO2 的 `/Odometry` 发布 `camera_init` 下的 **body 位姿**，自带 `loam_livox.rviz` 的 Odometry 显示使用 Axes 形状；TF 显示还可能同时画出 body、camera_init、base_link、map。因此看到红色 X 朝下，很可能是在看 body 或初始 camera_init 的轴。将 Fixed Frame 改为 map 只改变观察参考系，不会把所有被显示 frame 的 X 都变成车前方向。Axes 显示究竟属于谁，由它自己的 Reference Frame 决定。

为单独核对底盘坐标系，在 RViz 中：

1. Fixed Frame 保持 `map`，暂时取消原 Odometry、Axes 和 TF 的显示，保留点云和 XY Grid。
2. Add → Axes，命名为 `base_axes`，将 **Reference Frame 明确设为 `base_link`**，Length 可设 0.5 m、Radius 设 0.01 m。
3. 检查这组红色 X 是否沿车前且平行地面，蓝色 Z 是否垂直地面向上。屏幕朝下不一定等于物理朝地，需与网格平面比较。
4. 需要区分多组 TF 时再开启 TF 的 Show Names，并选择显示的 frames，避免几组轴重叠。

若单独显示的 base_link 仍相对地面明显翻转，检查 `rosrun tf tf_echo map base_link` 的旋转和两个静态 TF 的父子顺序、发布者，不能仅为让 body 的红轴水平而修改安装 TF。

### 3.6 把三维点云转换为 `/scan`

**终端 4：**

```bash
source ~/neupan_noetic_build/activate.bash
roslaunch neupan_cpp_ros1 livox_scan.launch \
  cloud_topic:=/cloud_registered_body target_frame:=base_link \
  min_height:=0.10 max_height:=1.00
```

转换节点先把 `body` 系 PointCloud2 变换到 `base_link`，按高度筛选，再按角度取最近点投影为二维 `sensor_msgs/LaserScan`。它不负责建图、外参标定或全局路径规划，也不自动去除全部地面和车体回波。高度上下限相对于 **base_link 原点**，这里沿用 launch 默认值，需要结合安装高度和避障对象核对；不能把倾斜的 `body` 直接当作地面对齐平面裁剪。

在检查终端中加载 NeuPAN 环境后执行：

```bash
source ~/neupan_noetic_build/activate.bash
rostopic type /scan
rostopic echo -n 1 /scan/header
rostopic hz /scan
rosrun tf tf_echo camera_init base_link
```

应得到 `sensor_msgs/LaserScan`、`frame_id: base_link` 和持续更新的数据。转换节点可能等待 `/scan` 订阅者才处理点云，`rostopic hz /scan` 或 RViz 的 LaserScan 显示即可触发订阅。在 RViz 中同时查看 FAST-LIO 点云、TF 和 `/scan`，检查二维点与障碍位置是否对应。

采用第 3.5.2 节的立装调平变换时，RViz Fixed Frame 使用 `map`，并补查 `rosrun tf tf_echo map base_link`；点云转换的 `target_frame` 仍为 `base_link`。

当前转换 launch 的距离范围是 0.5～5.0 m，FAST-LIO2 的 `preprocess/blind` 也是 0.5 m；近处盲区需在实车避障前核对，不能把被过滤的点当成近处没有障碍。

### 3.7 启动底盘驱动，接入路径与目标

**当前接线与接口已确认：底盘接在外置 PEAK PCAN-USB，其接口为 `can2`，驱动为 `peak_usb`；`can0/can1` 都关联 `ttyACM0`，属于 Innodisk。** 用户最新输出中 `can2` 为 DOWN。下面按本次 PEAK `can2` 接入，需将实际加载的驱动 YAML 中 `if_name: can0` 改成 `if_name: can2`；Innodisk 的 `emucd_64 -s7` 不用于配置 PEAK。完整映射与复核方法见 [接口选择说明](PCAN_CAN_CN.md)。重启或重插后再次确认接口名。

**这一步会主动发送电机使能请求。** 当前 `yhs_can_control_param.yaml` 中四个驱动电机和四个转向电机的 `motor_cmd_*_enable_*` 均为 `true`。驱动在 CAN 连接成功后通过 `SendControlBurst()` 发送启动帧，随后约每 10 ms 调用 `sendMotorCmd()` 持续发送使能状态；它不是只接收 CAN 反馈的节点。实际电机状态还需结合底盘控制模式与 `/motor_fb` 反馈确认。只做点云和 TF 检查可停留在第 3.6 节。

**启动前检查 USB-CAN 和 SocketCAN，尤其是新工控机或重新插拔 CAN 卡后。** 原始驱动 YAML 默认绑定 `can0`，本机需改为 `can2`；FW-mini V1.0.3 手册及驱动 README 指定波特率 500 kbit/s。先在驱动停止、遥控模式和桥接未使能的状态下检查：

```bash
ip -br link
ip -details -statistics link show can2
lsusb
```

若提示 `Device "can2" does not exist`，先处理 USB-CAN 识别或接口名称，不能继续把 ROS 启动失败当成遥控挡位问题：

- 查看实际接口是否为 `can1` 或其他 CAN 网络接口名。若确实更名，应同步修改实际启动包下 `config/yhs_can_control_param.yaml` 的 `if_name`，并在后续命令中使用相同名称。launch 会加载该 YAML，单独在启动前 `rosparam set` 可能被它覆盖。
- 驱动随附说明提到 PEAK PCAN-USB。若 `lsusb` 看到该设备但没有 CAN 网络接口，可检查内核 `peak_usb` 驱动：执行 `sudo modprobe peak_usb` 后重新检查接口；其他型号使用其实际支持 SocketCAN 的驱动，不套用 PCAN 模块名。
- 查看 `sudo dmesg | tail -n 60` 中的 USB/CAN 识别或掉线信息。仅出现 `/dev/ttyUSB*` 或厂商字符设备，不等于已经获得本节点需要的 SocketCAN 网络接口。

**2026-10-24 新工控机已确认存在 Innodisk USB Dual CAN。** `lsusb -v -d 04d8:0205` 的产品描述为 `innodisk USB Dual CAN`，由 `cdc_acm` 绑定为 `/dev/ttyACM0`；内核为 `5.15.0-139-generic`。初次排查时 `can0/can1/can2` 均不存在；后续已出现 UP 的 `can0`，但 RX 为 0，底盘反馈尚未验收。稳定链接为 `/dev/serial/by-id/usb-Microchip_Technology_Inc._innodisk_USB_Dual_CAN-if00`。具体板卡型号仍以标签为准，两个 CH340 串口不作为 CAN 设备使用。

现有 `FW-mini-ros` 是 ROS 底盘协议层，仍然可用。**通过 Innodisk 接入时使用 [Innodisk CAN 安装与接口创建步骤](INNODISK_CAN_CN.md)**：原厂 `emuc2socketcan` 模块与 `emucd_64` 工具将 tty 连接到 CAN 网络接口。已提取原厂 driver v3.10 / utility v3.6 包，并用隔离的 5.15.0-139 头文件编译通过。已有接口时先检查工具与通道状态，不重复加载或启动占用同一 tty 的进程。

Innodisk 工具以 `-s7` 设置两通道 500 kbit/s，随后**其两个接口都设为 UP**。命名 `can0 can1` 仅为示例：若 PEAK 已占用这些名字，应为 Innodisk 使用未占用的名称并同步替换后续命令。这条 Innodisk 路线不执行下面 PCAN 的 `type can bitrate` 命令，也不使用 `peak_usb` 或 `slcand`。不要把 `if_name` 改成 `/dev/ttyACM0`，ROS 底盘包需要的是 CAN 网络接口名。

**本机已确认 PEAK 是 `can2`，以下在工控机上配置该接口。** 此前在 Innodisk `can0` 上设置原生 CAN 位时序才得到 `Operation not supported`，本次将配置目标改为真正的 PEAK。也可使用 [带驱动检查的配置命令](PCAN_CAN_CN.md#2-配置已经确认的-peak-接口)。按顺序初始化，不将配置和启用命令用 `&` 并行执行：

```bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000
sudo ip link set can2 up
ip -details -statistics link show can2
timeout 10s candump can2
```

`candump` 只接收，不发送运动控制帧；本次用 `timeout` 在约 10 秒后结束观察。若缺少命令，安装 `can-utils`。PEAK 接口详情应包含 `link/can`、`bitrate 500000`，确认已设置 UP。查看是否有 `18C4D1EF`、`18C4DAEF`、`18C4DBEF` 等底盘反馈，而非仅观察 TX 计数；原始反馈仍为空时先保留接口状态与接收结果继续排查。若出现 BUS-OFF 或没有底盘数据，检查供电、CAN-H/L、波特率和终端电阻。

遇到 `>>bind dev_handler error!` 时，当前驱动尚未进入 `Initializing Chassis State...` 和发送启动使能帧阶段。它未检查 `SIOCGIFINDEX` 返回值、未打印 bind 的 errno，因此日志本身不能区分接口不存在、类型不对等原因。该错误与 TF 或 SWA 自动/遥控模式无关。CAN 接口若存在仍绑定失败，可用下面的独立检查获取具体系统错误；此代码不发送 CAN 帧：

```bash
python3 - <<'PY'
import socket
s = None
try:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(('can2',))
    print('SocketCAN bind can2 OK')
except OSError as exc:
    print('SocketCAN error:', repr(exc), 'errno=', exc.errno)
finally:
    if s is not None:
        s.close()
PY
```

**终端 5：** 第 2 节驱动补丁和构建完成、按适配器对应流程创建 CAN 接口并收到底盘反馈后，再启动现有驱动。只做雷达和 TF 检查时可暂不执行本步骤。

**先修改实际加载的驱动参数。** 激活环境后用 `rospack find yhs_can_control` 找到实际包，将其 `config/yhs_can_control_param.yaml` 中 `if_name: can0` 改为 `if_name: can2`，保留其他参数。修改 YAML 后重启节点即可，无需重新编译；启动前单独 `rosparam set` 可能被 launch 加载的 YAML 覆盖。

```bash
source ~/neupan_noetic_build/activate.bash
roslaunch yhs_can_control yhs_can_control.launch
```

launch 参数摘要应显示 `/yhs_can_control/if_name: can2`。随后在**终端 6**执行第 4 节的 `fw_mini_hardware.launch`，保持桥接禁用，先确认感知、路径及底盘反馈正常。

**遥控/指令模式与电机使能分开控制。** FW-mini V1.0.3 手册第 3.2.1 节规定：遥控器 SWA 向上为遥控模式，向下为指令控制模式。使用 ROS/CAN 控制车辆运动时需要切到指令控制模式（通常称自动挡）；仅启动驱动和查看反馈时，可先保持遥控模式。若遥控器通道映射已改动，以实际控制状态反馈核对，不能凭“模式二/三”代替 SWA 的控制权切换。

| 控制层 | 作用 | 当前行为 |
| --- | --- | --- |
| 驱动电机使能 | 驱动/转向电机使能请求 | 3.7 启动后按现有参数持续发送 true |
| 遥控器 SWA | 遥控器或 CAN 指令获得运动控制权 | 根据手册切换；ROS launch 不会替用户拨动或切换 SWA |
| `/fw_mini_bridge/enable` | 是否转发非零 NeuPAN 速度 | 默认 false；禁用时发送零速度，但不发送电机失能 |

首次测试顺序：保持遥控器开机、SWA 在遥控档、摇杆回中，准备好架空测试后启动底盘驱动；再启动默认禁用的桥接并确认 `/ctrl_cmd` 三个速度分量为零；随后切换 SWA 到指令控制档并核对反馈。方向、停车能力及输入有效性检查完成后，才执行第 4 节的桥接使能服务。切回遥控模式会交还遥控器控制权，不等于触发硬件急停。

当前消息包实际提供的检查话题如下，不使用旧 README 中与当前消息不一致的 `ctrl_fb_mode` 字段：

```bash
rostopic echo /motor_fb
rostopic echo /io_fb/io_fb_joypad_ctrl
rostopic echo /ctrl_cmd
```

按该手册 IO 协议，`io_fb_joypad_ctrl: true` 表示遥控控制，`false` 表示指令控制；同时结合遥控器状态与现场反馈确认。桥接使能前检查 `/ctrl_cmd` 的 `ctrl_cmd_x_linear`、`ctrl_cmd_y_linear`、`ctrl_cmd_z_angular` 都为零，不把 gear 字段误认为速度。

**2026-10-24 工控机反馈检查已通过：** 实际驱动参数为 `if_name: can2`，`/ctrl_fb` 的 498 条采样平均频率为 77.682 Hz（间隔 0.010～0.025 s）；`/motor_fb` 四个驱动与四个转向使能均为 true，电源重启为 false；`/io_fb` 可读，`io_fb_estop`、`io_fb_joypad_ctrl` 均为 false。这确认了 PEAK 原始 CAN 反馈到 ROS 消息的链路，不代表速度执行、控制权切换或实车闭环已验收。记录见 [CAN 接口与反馈结果](docs/results/can_adapter_mapping_20261024.json)。

`io_fb_joypad_online`、`io_fb_joypad_first` 是当前驱动额外解析的位；本次 V1.0.3 手册第 26 页未定义这两个位，不能仅凭它们为 false 就断定遥控器离线或优先级状态。下一步保持桥接禁用、摇杆回中并打开遥控器，观察 `rostopic echo /io_fb/io_fb_joypad_ctrl`：SWA 向上应反馈 true，向下应反馈 false。以现场拨杆与状态变化核对控制权，不将 `io_fb_estop: false` 当作已经完成实体急停测试。

若需要启动驱动读取反馈但不希望 ROS 发送使能请求，应在启动前把实际加载 YAML 中上述八项 enable 参数改为 false，再重新启动驱动；节点在构造时读取参数，运行后仅 `rosparam set` 不会更新当前发送值。这个设置只改变 ROS 侧请求，实际硬件控制仍以固件模式和反馈为准。本说明没有修改现有使能默认值。

`/initial_path` 应为 `nav_msgs/Path`，frame 与 `map_frame` 一致，至少两个点。使用 `map_frame:=camera_init` 时，路径 frame 就是 `camera_init`。也可在空旷区域将 RViz 的 2D Nav Goal 话题设为 `/neupan_goal`、Fixed Frame 设为 `camera_init`，生成短距离直线参考路径；使用地面对齐 `map` 时同步改为 `map`。

本实机入口没有启动全局 A*。交互仿真的 A* 使用虚拟地图，不能直接作为 FAST-LIO 实车地图规划器。复杂实车路线需由实际地图上的全局规划器发布 `/initial_path`；首轮先测试短直线、原地转向、缓弯。

## 4. 启动桥接，核对后使能

先从固件协议或已经验证的驱动命令确定 `ctrl_cmd_gear`。下面按第 3.5.2 节的车头立装方案使用地面对齐 `map`；若已确认 camera_init 本身地面对齐且未增加 map，则改回 `map_frame:=camera_init`。以下交互变量避免把测试代码中的任意挡位当成实车配置：

给定 V1.0.3 手册存在挡位描述差异：第 15 页 `0x18C4D1D0` 主协议表写 4T4D 为 `06`，第 33 页示例又将 `03` 称为运动学控制档；第 15 页的第三个控制分量为侧偏角，而当前驱动将其命名为 y 线速度。当前 diff 桥接固定第三分量为零，但有效运动挡位及组合 vx/wz 的执行效果仍需针对本车固件核对。八路电机使能为 true 或遥控器“模式二”均不能确定这个数值；在核对前不把 3 或 6 设置为通用默认值。

```bash
source ~/neupan_noetic_build/activate.bash
```

**下面的输入命令单独执行。** 等待提示出现后，只键入已核实的挡位整数并回车；输入期间不粘贴后面的 roslaunch 命令，否则它可能成为变量内容。

```bash
read -rp '输入已核实的固件 ctrl_gear 数值: ' FW_MINI_CTRL_GEAR
```

完成输入后检查变量，再启动。下面仅校验 4 bit 字段所需的整数格式，不验证该挡位是否适用于本车固件：

```bash
printf 'ctrl_gear=<%q>\n' "$FW_MINI_CTRL_GEAR"
if [[ "$FW_MINI_CTRL_GEAR" =~ ^([0-9]|1[0-5])$ ]]; then
  roslaunch neupan_cpp_ros1 fw_mini_hardware.launch ctrl_gear:="$FW_MINI_CTRL_GEAR" map_frame:=map base_frame:=base_link
else
  echo 'ctrl_gear 必须是 0 到 15 的纯十进制整数；请重新单独执行输入命令。'
fi
```

按操作员本次选择的 `6` 启动时，也可直接使用下面这条单行命令，不经过交互变量；这个例子不代表已经完成 gear=6 的实车运动验证：

```bash
roslaunch neupan_cpp_ros1 fw_mini_hardware.launch ctrl_gear:=6 map_frame:=map base_frame:=base_link
```

桥接默认 **DISABLED**，持续发零速度。若驱动订阅的话题不是 `/ctrl_cmd`，设置 `driver_command_topic:=实际话题`。确认该话题只有一个速度控制发布者，检查方向、单位和急停，再提供有效路径或目标。缺失 TF、scan 或路径时规划器发布安全停车，桥接拒绝使能；输入恢复后仍需手动使能，不会自动恢复运动。

```bash
rostopic info /ctrl_cmd
rostopic echo /ctrl_cmd
rostopic echo /fw_mini/applied_cmd_vel

# 在输入有效、方向/挡位和停车能力核对完成后执行
rosservice call /fw_mini_bridge/enable "data: true"

# 操作员停车并禁用；再次启动必须重新使能并收到新指令
rosservice call /fw_mini_bridge/enable "data: false"
```

`/neupan_emergency_stop` 为 true 时立即清零并禁用；false 仅解除信号，仍需重新使能。这里禁用的是速度桥接，不是驱动电机失能，也不能代替底盘硬件急停。桥接不接受使能前缓存的非零命令。上游规划命令超过 0.3 s 未更新时按最大加速度减速到零；桥接进程中断后，补丁后的驱动在其独立 0.3 s 超时后直接发零。

这两种超时不是同一种停车：正常指令断流使用斜坡；驱动断联、故障、操作员禁用与退出可立即清零。按最大 0.20 m/s 和 0.30 m/s² 计算，理想斜坡制动距离约 0.067 m，再加 0.3 s 检测过程最多约 0.06 m；机械响应、网络延迟和定位误差需另外实测。

### 4.1 查看 ctrl_cmd 时加载不到消息类

2026-10-24 工控机曾出现：`rostopic info /ctrl_cmd` 能列出桥接发布者与底盘订阅者，但 `rostopic echo` 报 `Cannot load message class for [yhs_can_msgs/ctrl_cmd]`。`info` 读取 master 元数据，`echo` 还需在当前终端的 Python 环境中导入消息类。桥接已经运行说明其运行环境可导入该类，优先修复**执行 rostopic 的诊断终端**，不据此重编译整个工程。

在报错的诊断终端执行：

```bash
source ~/neupan_noetic_build/activate.bash
source ~/work/mvp_ws/devel/setup.bash
python3 -c 'from yhs_can_msgs.msg import ctrl_cmd; print(ctrl_cmd._type)'
rostopic echo -n 1 /ctrl_cmd
```

导入应输出 `yhs_can_msgs/ctrl_cmd`，消息应包含 `ctrl_cmd_gear` 与三个速度字段。如果驱动工作空间不在默认位置，替换第二条 source 的路径。新的诊断终端需要重新加载环境；source 不会改变其他终端中已运行的节点。如果导入仍失败，再检查实际生成消息所在的工作空间和当前 Python 环境。

### 4.2 waiting for neupan initial path 与持续零指令

若启动时短暂出现 map 不存在，随后连续进入 `waiting for neupan initial path`，说明后续这些规划周期中 TF 查询和扫描新鲜度检查已通过，当前缺少参考路径。此时无需按旧的断链日志重复发布静态 TF；保留已有的实时输入，并补充路径或目标。

保持桥接默认禁用，在当前实车 RViz 中进行规划检查：

1. `Global Options → Fixed Frame` 设为 `map`，与规划器的 `map_frame` 一致。
2. 如果当前弹窗标题是 `Tool Type`，先点 Cancel。它用于添加工具插件，列表里不会有名为 neupan_goal 的工具；截图顶部已经存在 `2D Nav Goal`，直接配置它即可。
3. 若菜单中没有现成的 Tool Properties 面板，选择 **`Panels → Add New Panel → rviz → Tool Properties → OK`**。`Panels → Tools` 勾选项控制顶部工具栏；`Delete Panel` 是删除面板入口，均不是目标话题配置位置。
4. 在新出现的 **Tool Properties** 面板中展开 **`2D Nav Goal → Topic`**，双击 Topic 的值，手工输入 **`/neupan_goal`** 并回车。名称是单数 goal，没有末尾 s；不需要在插件列表或话题下拉选项中找到它。常见原值 `/move_base_simple/goal` 或 `goal` 需替换成本节点订阅的目标话题。
5. 点击顶部已有的 `2D Nav Goal`，在车前确认空旷、直线可达的位置按住左键，拖出目标朝向后松开，选择短目标，例如距离当前底盘约 0.5 m。本实机 launch 未启动仿真中的全局 A*，先在静止、桥接禁用时检查参考路径和规划轨迹。
6. 添加 Path 显示，Topic 分别选择 `/neupan_initial_path` 与 `/neupan_plan`。正常收到目标时，节点会记录 `set neupan goal`。配置完成后用 `File → Save Config As` 保存自己的 RViz 配置，方便下次复用。

可用 `rostopic info /neupan_goal` 核对类型为 `geometry_msgs/PoseStamped`、发布者包含 RViz、订阅者包含 `/neupan_node`。需要捕获一次目标消息时，先在终端执行 `rostopic echo -n 1 /neupan_goal`，再回到 RViz 点击并拖动目标；目标消息在选点时发布，不应在选点之后才启动 echo 等待旧消息。

也可由实际全局规划器发布 `/initial_path`（nav_msgs/Path，frame=`map`，至少两个点）。注意 `/initial_path` 是路径输入，`/neupan_initial_path` 是规划器的参考路径显示输出。

诊断终端可检查：

```bash
rostopic echo -n 1 /neupan_initial_path
rostopic echo -n 1 /neupan_emergency_stop
rostopic echo -n 1 /neupan_cmd_vel
rostopic echo -n 1 /fw_mini/applied_cmd_vel
```

缺少路径时，规划器的 `halt()` 会发布零速度和 `/neupan_emergency_stop: true`，桥接收到后禁用并拒绝使能；不能用反复调用 enable 来代替路径输入。路径、TF、scan、求解和几何检查均正常、停车信号解除后，桥接仍保持禁用，需按前述实车检查流程手动使能。即使 `/neupan_cmd_vel` 已出现非零规划指令，禁用时 `/ctrl_cmd` 的三个速度分量和 `/fw_mini/applied_cmd_vel` 仍应为零。

`/fw_mini/applied_cmd_vel` 记录的是约 50 Hz 的桥接输出指令，不是编码器或实测速度；持续零值符合当前禁用状态，不能据此判断车辆是否具备执行指令的能力。当前 CAN/ROS 反馈检查已通过，gear=6 的速度执行与停车效果仍需实车验收。

## 5. 日志与实机验收

桥接默认保存到 `~/neupan_hardware_results/bridge_时间/`：参数快照、挡位/单位说明及每个周期的 `commands.csv`。同时记录 target、applied、墙钟 dt、变化率、使能、急停、是否新鲜、`stop_override` 和驱动字段。急停等越过正常变化率的行为保留在日志里，不隐藏。

```bash
rosbag record -O fw_mini_test.bag \
  /neupan_cmd_vel /fw_mini/applied_cmd_vel /ctrl_cmd /ctrl_fb /odom \
  /scan /tf /tf_static /neupan_diagnostics /neupan_emergency_stop
```

先架空验证前进/后退与正负转向、挡位、角速度换算、禁用和断流停车，再落地低速验证短直线、缓弯和停车距离，最后再验证避障走廊。对比已下发指令与 `/ctrl_fb`、里程计/定位轨迹；实车反馈角速度也需按驱动协议统一为 rad/s。规划指令、CAN 量化后的指令、实测速度/加速度分别计算，不能把软件限幅当成车辆动力学验收。

## 6. 不接硬件的接口复测

运行 ROS master，消息包已构建并激活后：

```bash
python3 neupan_cpp_ros1/test/test_command_guard.py
python3 neupan_cpp_ros1/test/fw_mini_bridge_test.py \
  --output ~/neupan_sim_results/fw_bridge_transport
```

测试创建独立 `/fw_bridge_selftest_PID/...` 话题，不发布真实 `/ctrl_cmd`；测试中挡位 2 只是校验字段传输，不能作实车挡位依据。

驱动管道测试直接链接补丁后的真实驱动代码，仅把帧写进 POSIX 管道，不打开 CAN：

```bash
cmake -S hardware -B /tmp/fw_mini_watchdog_test \
  -DFW_MINI_DRIVER_SOURCE="$HOME/work/mvp_ws/src/FW-mini-ros/yhs_can_control"
cmake --build /tmp/fw_mini_watchdog_test -j2
/tmp/fw_mini_watchdog_test/watchdog_pipe_test
```

本次上述 ROS1 接口和管道测试均通过；物理 CAN 通信、底盘执行及真实避障尚未完成验收。
