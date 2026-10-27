# diff 控制版本归档说明

> 2026-10-27 更新：主分支现已完成 omni C++/ROS1 移植及工控机仿真复测。本文保留 2026-10-26 的 diff 归档经过；归档分支/标签保持原快照，当前使用入口见 [README.md](README.md)。

日期：2026-10-26。目标仓库：[mvp-inspection-robot](https://github.com/sustc-xia-explorer/mvp-inspection-robot)。

## 1. 原因与方式

用户已向 FW-mini 厂商确认：本车不能按此前设想锁定后轮使用 diff，需要转为全向底盘方式。因此冻结当前 C++ / ROS1 diff 执行方案，保留其研究与复现价值，后续不继续作为本车实机运动控制方案。

这里记录的是本车厂商约束与项目路线调整，并不将后轮锁止定义成所有机器人使用 unicycle/diff 数学模型的必要条件。物理机构、固件可接受指令和规划模型需要分别验证。

- 归档分支：[`archive/diff-ros1-20261026`](https://github.com/sustc-xia-explorer/mvp-inspection-robot/tree/archive/diff-ros1-20261026)。
- 归档标签：[`diff-ros1-20261026`](https://github.com/sustc-xia-explorer/mvp-inspection-robot/tree/diff-ros1-20261026)。
- 文件清单：[diff_archive_manifest_20261026.json](docs/results/diff_archive_manifest_20261026.json)。
- 本次检查：[diff_publication_checks_20261026.json](docs/results/diff_publication_checks_20261026.json)。

代码保持原目录布局，已有构建和实验路径仍有效。分支/标签保存归档快照，主分支 README 标识冻结状态；没有搬移代码或修改参数来伪装成 omni。

需要读取归档时，可克隆到新目录：

```bash
git clone --branch archive/diff-ros1-20261026 https://github.com/sustc-xia-explorer/mvp-inspection-robot.git mvp-diff-archive
```

## 2. 保存范围

| 内容 | 归档范围 |
| --- | --- |
| C++ 核心 | Eigen DUNE、NRMP/OSQP、PAN、路径处理、diff 模型 |
| 变化率控制 | 首拍跨周期约束、输出投影、执行限幅和原有回归测试 |
| ROS1 | TF、scan、路径/目标、轨迹、诊断、停车信号 |
| 仿真 | 理想 diff 底盘、静态 A*、RViz、7+14 场景及历史工控机报告 |
| 模型 | FW-mini 导出权重、训练辅助脚本与已有训练结果 |
| 硬件资料 | diff 桥接、驱动超时补丁、CAN/消息环境/TF/RViz 排查记录 |

`FW-mini-ros` 和 `ws_livox` 是外部工作空间，仓库保存驱动补丁及接入资料，不混入它们的整套源码、build/devel、第三方适配器 ZIP 或完整厂商 PDF。

## 3. 已完成与未完成的验证

历史 7 场景、14 扩展场景和完整走廊的结果保留，适用对象仍为报告中的理想 diff 模型。2026-10-26 从当前源码重新构建，现有 C++ 测试 4/4、Python 执行守卫测试 4/4 通过；未重新宣称跑过实车或整套历史场景。

原样保存的 `libneupan/build.sh` 实际是一行带 `<length>/<width>` 占位符的权重导出示意命令，不是可直接执行的构建脚本，不能通过 bash 语法检查。本次不改写这份历史文件；实际 Noetic 构建入口仍为 `tools/build_noetic.sh`，归档检查记录区分这两者。

用户提供的工控机证据包括：

- PEAK 对应 can2，底盘线接在 PEAK 上；can0/can1 为 Innodisk。
- `/ctrl_fb` 的 498 条采样平均 77.682 Hz，电机八路使能反馈为 true，IO 可读。
- `/ctrl_cmd` 为 gear=6、三个速度分量为零，桥接发布者与底盘订阅者存在。
- RViz `/neupan_goal` 已能被节点接收。

这些证据不能证明本车 diff 实机闭环成功。接受目标不等于轨迹被正确执行；零速度消息不能验证 gear=6 的非零动作，也不能区分 CAN 第三分量的侧偏角/横向速度语义。

以下仍未验收：实际速度方向与组合运动、控制权切换、实体停车、真实转向限制与切换时间、最终外参与完整 footprint、全向循迹避障、同机轻量化加速倍数。

## 4. 后续边界

本次只上传现有版本、文档与评估，没有修改控制实现、运动模型、launch 或 YAML 控制数值。`FW_MINI_HARDWARE_CN.md` 保留的使能命令为历史资料，不作为新全向方案的实机步骤。

后续见 [OMNI_ADAPTATION_CN.md](OMNI_ADAPTATION_CN.md)。DUNE、ROS1 基础设施及验证工具具有复用价值，但历史 diff 通过率和耗时不能直接作为新模型结果。
