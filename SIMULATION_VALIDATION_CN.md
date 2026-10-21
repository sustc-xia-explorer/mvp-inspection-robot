# 实机前验证：模型导出、算法回放与 ROS1 闭环仿真

先验证移植数值，再验证闭环行为，最后验证真实传感器/坐标系和底盘。通过数值测试不代表路径能收敛，虚拟车通过也不代表真实四轮转向底盘已验证。

需要在 RViz 点选终点、观察 FW-mini 连续绕障时，使用 [交互仿真说明](RVIZ_SIMULATION_CN.md)。工控机拷回结果的逐项结论见 [工控机结果分析](IPC_RESULTS_ANALYSIS_CN.md)。2026-10-21 后两个规划入口统一默认加载 `fw_mini_balanced.yaml`，开发机通过 7/7 原场景、14/14 扩展场景及完整走廊，详见 [调参报告](PARAMETER_TUNING_CN.md)。本页的 `fw_mini.yaml` / Python 对照和第 8 节失败结果属于保留的原基线。

## 1. 原 test 目录可以用到哪里

`neupan_cpp_ros1/test` 中原有的 `smoke_test.py`、`closed_loop_test.py`、`path_overshoot_test.py`、`sentry_smoke_test.py`、`astar_test.py` 仍使用 ROS2 的 rclpy。这些是参考代码，不能直接在 Noetic 运行。

原 smoke/sentry 测试主要检查收到速度消息，零速度也可能通过；原 overshoot 测试虽然打印 stopped，却没有把它纳入通过条件；部分脚本发 `/goal_pose`，而本 ROS1 节点接收 `/neupan_goal`。因此只改 import 不能完成验证。

原 C++ `test_dune`、`test_nrmp`、`test_replay` 可以继续使用。test_replay 会同步 Python 的 nominal 控制序列，是开环数值回放；其内置数据来自原默认机器人，而不是你训练的 FW-mini。原回放中的统一动作尺度和开发机 5 ms 时延阈值也不能直接作为 FW-mini 实车验收标准。

工控机曾因回放均值 5.00924 ms 略高于原源码写死的 5 ms 而中止构建，日志没有数值断言失败。现已保留全部动作、距离和 stop/arrive 数值检查，将耗时默认设为报告项。若要单独执行 5 ms 性能验收，可显式设置预算：

```bash
NEUPAN_REPLAY_MAX_MEAN_MS=5 ctest \
  --test-dir ~/neupan_noetic_build/libneupan -R '^test_replay$' -V
```

预算必须是有限正数；超过预算仍会失败。不设置该变量时不以耗时决定数值回归是否通过，但这也不代表通过了实时性验收。正式性能结论仍需 FW-mini 实际模型、相同输入和同一工控机的对照数据。

新增文件：

- `scripts/simulation_test.py`：7 个场景，共用世界、扫描、虚拟车和独立碰撞检测，支持 ros/core/python 三种后端。
- `libneupan/tools/sim_backend.cpp`：纯 C++ 核心后端，构建目标 `neupan_sim_backend`。
- `tools/check_dune_export.py`：检查实际 .pth 与 .bin 的 MLP 前向是否一致。
- `tools/compare_simulations.py`：生成 Python/C++ 自由闭环轨迹对照图和表。

这里 scripts 指 `neupan_cpp_ros1/scripts`。原 ROS2 测试保留，不会被当成 ROS1 测试自动安装。

## 2. 将测试补充包装到工控机

补充包 `neupan_simulation_update.tar.gz` 不包含模型，也不覆盖已有的 `config/fw_mini.yaml`；当前包同时提供交互仿真的新增 YAML，原权重与回归参数保留。进入包含 libneupan、neupan_cpp_ros1、tools 的源码根目录，例如：

```bash
cd ~/work/neupan_noetic_bundle/neupan_cpp
cp libneupan/CMakeLists.txt libneupan/CMakeLists.before_sim.txt
cp neupan_cpp_ros1/CMakeLists.txt neupan_cpp_ros1/CMakeLists.before_sim.txt
tar -xzf ~/work/neupan_simulation_update.tar.gz -C .
bash tools/build_noetic.sh all
source ~/neupan_noetic_build/activate.bash
```

修改当前电脑源码也已经完成；若是完整复制更新后的源码，直接重新执行 build_noetic.sh all 即可。首次出现编译错误应看错误本身，不要靠调宽仿真阈值解决。

绘图依赖 matplotlib，仿真场景配置依赖 PyYAML：

```bash
sudo apt install python3-yaml python3-matplotlib
```

没有 matplotlib 时仍会输出 CSV 和 JSON。

## 3. 第一层：核心数值回归和实际模型导出检查

在已完成构建的工控机执行：

```bash
source ~/neupan_noetic_build/activate.bash
cd ~/neupan_noetic_build/libneupan
ctest --output-on-failure
```

再在有 Python NeuPAN 环境的开发电脑检查你自己的 FW-mini 模型。以下命令在 C++ 源码根目录执行；需要先在那里执行 `bash tools/build_noetic.sh core` 构建 C++ 测试后端：

```bash
source ~/NeuPAN/.venv/bin/activate
python tools/check_dune_export.py \
  --binary ~/neupan_noetic_build/libneupan/neupan_sim_backend \
  --config neupan_cpp_ros1/config/fw_mini.yaml \
  --model neupan_cpp_ros1/models/fw_mini.bin \
  --checkpoint tools/model/fw_mini/model_5000.pth \
  --output ~/neupan_sim_results/fw_dune_export.json
```

它用 1000 个相同 float32 输入点比较 PyTorch 和 Eigen MLP，包括近车体和正常扫描范围的点。默认最大绝对误差阈值 1e-4。这个检查验证权重导出与前向实现，不验证神经距离是否准确，也不替代 NRMP 回放。

## 4. 第二层：先跑纯核心虚拟闭环

核心模式不需要 roscore，按数学时间推进，能很快发现不收敛、碰撞、越界和终点不停等问题：

```bash
source ~/neupan_noetic_build/activate.bash
cd ~/work/neupan_noetic_bundle/neupan_cpp
python3 neupan_cpp_ros1/scripts/simulation_test.py \
  --backend core --cases all --output ~/neupan_sim_results/core_fw
```

默认使用包内 fw_mini.yaml/fw_mini.bin。如果模型放在其他位置，显式传 `--config` 和 `--model`。若构建目录不是默认路径，用 `--binary /实际构建目录/libneupan/neupan_sim_backend`。

仿真脚本会保留已有结果。如果 `--output` 目录已存在，自动改用带时间戳的新目录，例如 `core_fw_20261019_201500`，同一秒重名时再追加编号。终端开头的 `Writing results to:` 显示本次实际目录。旧版脚本出现 `FileExistsError` 时，仿真尚未开始，可以更新脚本，或直接指定一个新目录：

```bash
python3 neupan_cpp_ros1/scripts/simulation_test.py \
  --backend core --cases all \
  --output "$HOME/neupan_sim_results/core_fw_$(date +%Y%m%d_%H%M%S)"
```

| 场景名 | 检查内容 |
| --- | --- |
| straight | 2 m 直线，抵达后稳定停车 |
| heading_north | 初始朝向 90°，检查坐标方向与运动一致 |
| turn_left | 给定直角折线路径，检查转弯和终点收敛 |
| sparse_stop | 0、2、4 m 三个稀疏路径点，检查终点超调及停车 |
| corridor | 走廊两侧距车体各约 0.35 m，检查振荡、卡住和碰撞 |
| detour | 跟踪已经绕开圆障碍的参考路径；不是测试自动全局绕路 |
| blocked_wall | 目标被宽墙阻挡，允许未到达，但必须靠近后安全停住 |

虚拟车按理想 diff 运动学执行原始规划输出。每个控制周期内分 5 次检查完整矩形车体与圆/墙的真实距离，计算独立于 DUNE。场景并没有模拟轮组切换、轮胎打滑、CAN 延迟或实际动力学。

## 5. 第三层：ROS1 全链闭环

已有 roscore 可继续使用；否则另一个终端执行 roscore。仿真自行发布带唯一前缀的 TF、扫描和参考路径，速度仅回到虚拟车。它不发布全局 /cmd_vel 或 /ctrl_cmd。

```bash
source ~/neupan_noetic_build/activate.bash
rosrun neupan_cpp_ros1 simulation_test.py --cases all \
  --output ~/neupan_sim_results/ros_fw
```

先快速验证几个场景也可以：

```bash
rosrun neupan_cpp_ros1 simulation_test.py \
  --cases straight,heading_north,sparse_stop \
  --output ~/neupan_sim_results/ros_basic
```

ROS 模式使用实际墙钟与 node 定时器，完整 7 场景需数分钟。它加入 0.2 m、0.05 m、0.35 rad 的二维模拟传感器外参，检查 TF 平移、yaw 和扫描处理。这里不模拟竖装 Mid-360S 的三维投影，后者仍需用真实点云单独验证。

第 6 步已有 `smoke_test.py` 继续负责首次扫描前和扫描丢失后的零速度检查。本场景脚本侧重轨迹、抵达、车体碰撞和障碍约束，两者一起使用。

## 6. 用 Python 做相同场景的对照

在开发电脑的 Python NeuPAN 环境运行；配置和 .pth 必须对应 C++ 的配置与 .bin：

```bash
source ~/NeuPAN/.venv/bin/activate
python neupan_cpp_ros1/scripts/simulation_test.py --backend python \
  --checkpoint tools/model/fw_mini/model_5000.pth --cases all \
  --output ~/neupan_sim_results/python_fw

python tools/compare_simulations.py \
  ~/neupan_sim_results/core_fw ~/neupan_sim_results/python_fw \
  --output ~/neupan_sim_results/comparison_fw
```

对照命令的两个输入路径要使用本次仿真实际打印的目录；对照工具的 `--output` 仍需指定尚不存在的新目录。

相同场景、相同虚拟车并不意味着两版每帧状态完全相同；小的动作差异会经闭环反馈累积。因此对照工具报告的是自由闭环轨迹差，不是“完全相同输入下的求解误差”。判断求解移植误差要结合第一层固定输入/nominal 回放。

若 Python、C++ 同场景都失败，先检查共有的模型、参考路径和参数；若只有 C++ 失败，再重点定位移植、求解器和 ROS 时序差异。两者都通过还需核实误差、真实间距、速度变化，不能仅看 PASS。

## 7. 输出与初始验收阈值

每次运行保存 config_snapshot.yaml、run.json（含模型哈希）、总 summary.csv/json。每个场景有 trajectory.csv、trajectory.png、summary.json、backend.log。

CSV 包含时间、位姿、纵向/角速度、路径距离误差、终点误差、独立车体间距、障碍点数、求解/抵达状态、DUNE 距离。core/python 还记录完整 planner forward 的内部计算时间；ROS 模式不把整段通信时间冒充算法耗时。

初始工程阈值：终点误差 <=0.15 m、路径距离最大值 <=0.35 m/RMS <=0.15 m、终点超调 <=0.15 m、有障碍场景真实间距 >=0.03 m。路径距离是点到有限折线段的最短距离，包括端点之外的距离。抵达须由规划器报告 arrive 并保持至少 1 s 的低速度；封路场景则检查未误报抵达且停车至少 2 s。速度上限采用 YAML。碰撞一律失败。

这些是发现回归的起步阈值，不是论文最终指标或实车安全认证。最大实际命令加速度也会记录；它当前不作为单独 PASS 门槛，因为需要区分规划器的预测约束与底盘输出端限加速度是否实现。不要仅调整阈值让失败消失。

失败场景仍会保存结果并继续下一个，最终退出码 1 表示至少一项验收失败，不一定是安装或程序崩溃。要把 backend.log 中的异常与 summary.json 的验收失败原因分开查看。

## 8. 2026-10-19 的实际结果

开发电脑 Ubuntu 22.04，使用当前 FW-mini 配置及实际生成的 model_5000.pth/fw_mini.bin，进行了 core/Python 后端的理想闭环。没有将这些结果写成工控机 ROS1 新场景已通过。

- 原核心 test_dune/test_nrmp/test_replay：3/3 通过。
- 独立扫描/碰撞几何测试：6/6 通过。
- 实际 FW-mini MLP 对照：1000 点，最大绝对误差 1.81198e-5，RMS 3.86286e-7，阈值 1e-4，通过。
- core 和 Python 都为 5/7：直线、90°初始朝向、稀疏终点、给定绕障路径、封路停车通过。
- turn_left：两版均停在终点侧方约 0.136 m，未报告 arrive；50 s 内未完成。
- corridor：两版均出现振荡和前进缓慢，70 s 内未完成。C++ 离目标约 2.040 m，Python 约 1.746 m。两版均无车体碰撞。

因此当前参数尚不适合直接开展实车循迹验收。共同失败尚未证明是移植错误；应保持两版同样配置，优先检查直角参考路径的连续性、终点收敛、走廊可用间距与避障代价、控制量跳变，定位后再回归测试。不要只把 arrive_threshold 调大来消除转弯失败。

结果位于当前开发电脑 `work/neupan_sim_results`。这些是正确性测试，部分并行执行，不能用其中的时间直接声称轻量化加速倍数；正式性能对照应在同一 i5 上按相同输入串行运行、预热并统计。

后续再进行真实点云/TF回放观察、低速控制接口验证及实车测试。固定 rosbag 回放能检查感知与规划输出，却不会响应模拟速度改变机器人状态，因此不能替代闭环跟踪验证。
