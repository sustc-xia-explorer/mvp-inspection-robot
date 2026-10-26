# Python NeuPAN omni 与 FW-mini 全向底盘适配评估

日期：2026-10-26。本次只审查源码和方案，没有修改控制实现或执行实车全向指令。

## 1. 结论

**可以将 Python omni 作为固定车体航向、任意方向平移的算法起点，但不能直接接到当前 FW-mini 驱动上。** 完整的双阿克曼/四轮独立转向导航还需适配预测模型、坐标系、CAN 语义和轮组约束。

厂商已确认本车不采用此前后轮锁止的 diff 方案，本项目据此归档旧执行路线。四轮独立驱动不等于麦克纳姆轮底盘；普通前轮 Ackermann 模型也不能直接代表双阿克曼/四轮独立转向。

首要问题是固件接受 `vx/vy/wz`，还是速度大小、侧偏角和角速度，以及这些量能否在同一模式下同时执行。已有遥控横移和 CAN 零速度通信记录不能代替该确认。

## 2. 核对版本

官方仓库：[hanruihua/NeuPAN](https://github.com/hanruihua/NeuPAN)，本次独立下载提交 `d225c3862654ade30ccd19455997f615a689a670`。

本机 `~/NeuPAN` 提交 `f5ae6d848b54aeae0340af1eb49e3a809dea00a5` 的以下关键文件与官方版本完全一致：

- [robot.py](https://github.com/hanruihua/NeuPAN/blob/d225c3862654ade30ccd19455997f615a689a670/neupan/robot/robot.py)：变量、代价、约束、线性化。
- [neupan.py](https://github.com/hanruihua/NeuPAN/blob/d225c3862654ade30ccd19455997f615a689a670/neupan/neupan.py)：内部控制到最终 action 的转换。
- [initial_path.py](https://github.com/hanruihua/NeuPAN/blob/d225c3862654ade30ccd19455997f615a689a670/neupan/blocks/initial_path.py)：名义运动预测。
- [obs_point_net.py](https://github.com/hanruihua/NeuPAN/blob/d225c3862654ade30ccd19455997f615a689a670/neupan/blocks/obs_point_net.py)：DUNE 点网络。

源码哈希及直接执行原始 NumPy 预测函数的两个例子见 [核对记录](docs/results/omni_review_20261026.json)。这不是完整求解器、训练或实车测试。

## 3. 原始 omni 的真实含义

| 项目 | 源码行为 |
| --- | --- |
| 状态 | `[x, y, theta]` |
| 内部控制 | `[v, phi]`：平移速度大小与平移方向角 |
| 最终输出 | `[v*cos(phi), v*sin(phi)]`，即平面 vx/vy |
| 车体 yaw | 预测中保持不变，没有第三个独立角速度控制量 |
| 状态跟踪代价 | omni 分支对 x/y 计跟踪代价，不以此项跟踪 yaw |

当状态和障碍点在 map 系时，模型为：

```text
x_next = x + dt * v * cos(phi)
y_next = y + dt * v * sin(phi)
theta_next = theta
```

phi 是平移方向，不是车体 yaw 或角速度。它允许改变移动方向而保持车身朝向，适用于蟹行/斜移的初步研究；不提供完整的独立平移与机身旋转控制。

代码中 `max_speed[1]` 限制内部方向角 phi，`max_acce[1]*dt` 限制相邻 phi 的变化。不能把历史 diff 的两维速度/角速度和加速度参数原样理解为 omni 的 vx/vy 限制，也不能仅按 README 的 angular speed 命名判断。

原始预测函数的核对例子：速度 0.2 m/s、方向 pi/2、dt=0.1 s 时，y 增加 0.02 m，yaw 不变。

## 4. 与实车能力的匹配

给定《FW-mini 用户使用手册 V1.0.3》描述四轮独立转向/驱动、双阿克曼、横移、斜移和原地转向；表 2-1 给出轴距 350 mm、轮距 250 mm、轮胎直径 140 mm。外廓 495×360 mm 与轴距/轮距不是同一组参数。

| 需求 | 原始 omni 的适用范围 |
| --- | --- |
| 固定车身 yaw 的前后、左右、斜向移动 | 可表达理想运动，仍需证明固件可执行 |
| 位置沿折线变化、车身朝向不变 | 可作为候选，需验证矩形外廓和转向过渡 |
| 同时平移与独立转动机身 | 原始二维控制模型不能直接表达 |
| 原地改变车体 yaw | 没有独立角速度控制量 |
| 四轮转角、转向速率、模式切换 | 原始模型未包含这些约束 |

最小验证对象是固定 yaw 的全向平移，前提是固件支持任意平移方向，而非只能在前后与正横移之间离散切换。双阿克曼、蟹行与原地转向是否可同时组合，不能由四个驱动电机或遥控模式编号推断。

## 5. 必须适配的接口与约束

### 5.1 速度坐标系

输出的 vx/vy 与状态 x/y 使用同一平面坐标系。规划在 map、命令要求 base_link 时，应转换：

```text
vx_body =  cos(theta)*vx_map + sin(theta)*vy_map
vy_body = -sin(theta)*vx_map + cos(theta)*vy_map
```

例如车身 yaw=pi/2、要求沿 map 的 +X 以 0.2 m/s 平移，对应车体系约 `[0,-0.2]`。原始预测函数的运行例子验证了它不会自动把车体系朝向混入平移方向。

本机 `neupan_ros2/neupan_node.py::generate_twist_msg()` 把二维 action 第二项固定写入 `Twist.angular.z`，没有 omni 分支。仅改 YAML 会把 vy 当作角速度。当前 C++/ROS1 桥接也只支持 diff，拒绝非零 `linear.y`。两个现有 ROS 包都不能直接作为本车全向接口。

### 5.2 CAN 字段不是仅凭名称映射

当前消息叫 `ctrl_cmd_x_linear / ctrl_cmd_z_angular / ctrl_cmd_y_linear`；手册第 15 页对 `0x18C4D1D0` 却定义车体线速度、角速度和**侧偏角**，第三量为 0.01 度/bit，当前驱动第三量则乘 1000。主表 gear=6，第 33 页示例 gear=3，驱动 README 又有另一套挡位和消息字段。

这些资料不一致，不能仅凭 `y_linear` 的名字把横向速度直接打包。当前第三分量为零，零值通信不能分辨其物理含义。

应向厂商确认固件、ID、gear、速度是纵向分量还是合速度、侧偏角单位与正方向、组合动作和模式规则。若明确输入为合速度与侧偏角，才可考虑模长/atan2 转换；若速度定义不同，映射也不同。

### 5.3 四轮可达性与转向过程

四轮独立转向的理想无侧滑关系，可用轮心在车体系中的位置 `(xi,yi)` 表示：

```text
wheel_vx_i = vx_body - wz*yi
wheel_vy_i = vy_body + wz*xi
wheel_angle_i = atan2(wheel_vy_i, wheel_vx_i)
wheel_speed_i = sqrt(wheel_vx_i^2 + wheel_vy_i^2)
```

这是后续设计的运动学关系，不代表当前固件提供逐轮指令接口。还需转向范围/速率、轮速上限、反向轮速选择、近零速方向处理与到位判定。高层固件已经解算的部分优先复用，不擅自替换成逐轮控制。

理想 omni 可瞬时改变移动方向，真实轮组需要转向时间。限制 vx/vy 变化不自动满足轮组转向限制。模式切换需要时序与反馈，轮心坐标也须以真实 base_link 原点和实际轴距/轮距测定。

## 6. 后续模型路线

起步可研究固定 yaw 的全向平移：正确转换坐标和协议，验证前后、左右、斜移与停止。如果固件只有离散横移/转弯模式，就需要模式切换设计，不能按理想连续 omni 直接执行。

若巡检还需独立控制机身朝向，候选模型是状态 `[x,y,theta]`、车体系控制 `[vx,vy,wz]`：

```text
x_dot = cos(theta)*vx - sin(theta)*vy
y_dot = sin(theta)*vx + cos(theta)*vy
theta_dot = wz
```

还应加入本车可达速度、转向/模式约束。C++ 控制维度、QP、参考轨迹/热启动、首拍限制、执行守卫、ROS/CAN 转换与仿真底盘都需适配，只放开 `linear.y` 不足以实现。普通单轴距前轮 Ackermann 模型也不直接覆盖完整四轮独立转向约束。

## 7. DUNE 是否重训

DUNE 网络输入是车体局部障碍点，训练标签由几何 G/h 决定，没有将 diff/omni 控制向量作为输入。**仅切换运动学不必然要求重训。** 若 footprint、原点、坐标轴与元数据一致，现有 `fw_mini.bin` 有复用可能，仍需复测距离误差与部署前向一致性。

若加入突出雷达、轮组转向外扫或更改参考点导致几何变化，应重新训练/导出或采用经过验证的包络。预测 yaw 必须与实际车体姿态一致，不能加一个未进入预测的航向控制器后直接沿用旧避障结论。

## 8. 下一阶段前需确定的资料

1. 本车固件与准确 CAN 协议，尤其第三控制分量和合法 gear。
2. 同一模式下是否支持组合平移/旋转，还是要切换阿克曼、蟹行和原地转向。
3. 轮组角度/转向速率/轮速限制、模式到位、停车及控制权规则。
4. 完整外廓、轮心位置、安装外参与命令参考点。
5. 任务允许固定机身朝向，还是必须同时跟踪 yaw。

确定后再选固定 yaw 起步或带四轮约束的三自由度模型，先做数值与仿真对照，再实现 C++ 和实机接口。本次没有实施这些改动。
