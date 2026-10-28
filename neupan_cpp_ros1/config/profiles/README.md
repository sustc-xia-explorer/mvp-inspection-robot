# 规划参数配置命名与位置

本目录按运行核心区分参数，原文件保留。带版本号文件是冻结快照；原入口不会自动跟随快照变更。修改哪一份，运行时就通过 config_file 显式选择哪一份。

| 编号 | 中文名称 | 核心 | T / PAN | 文件 | 说明 |
|---|---|---|---|---|---|
| FW-OMNI-FULL-V1 | 本地全向完整平滑版 | ros1 | 15 / 4 | [fw_mini_omni_full_v1.yaml](ros1/fw_mini_omni_full_v1.yaml) | 历史完整平滑快照；T15/PAN4 |
| FW-OMNI-LIGHT-V1 | 本地全向小计算基准 | ros1 | 10 / 2 | [fw_mini_omni_light_v1.yaml](ros1/fw_mini_omni_light_v1.yaml) | T10/PAN2；连续急弯低速 3.1 s |
| FW-OMNI-LIGHT-TURN-V2 | 本地全向小计算急弯调优版 | ros1 | 10 / 2 | [fw_mini_omni_light_turn_v2.yaml](ros1/fw_mini_omni_light_turn_v2.yaml) | 当前推荐默认；原 ROS1 核心，已完成工控机复测 |
| FW-OMNI-CORRIDOR-V1 | 本地全向旧走廊平滑版 | ros1 | 15 / 4 | [fw_mini_omni_corridor_v1.yaml](ros1/fw_mini_omni_corridor_v1.yaml) | 加入完整转弯速度包络之前的版本 |
| FW-OMNI-LEGACY-V0 | 本地全向初版 | ros1 | 10 / 3 | [fw_mini_omni_legacy_v0.yaml](ros1/fw_mini_omni_legacy_v0.yaml) | 早期 omni 参数，供回溯 |
| FW-DIFF-BALANCED-ARCHIVE | FW 差速均衡归档版 | ros1 | 10 / 2 | [fw_mini_diff_balanced_archive.yaml](archive/fw_mini_diff_balanced_archive.yaml) | 旧 diff 基准；厂商已确认当前车不按锁后轮 diff 使用 |
| FW-DIFF-INITIAL-ARCHIVE | FW 差速初始归档版 | ros1 | 10 / 2 | [fw_mini_diff_initial_archive.yaml](archive/fw_mini_diff_initial_archive.yaml) | 旧 diff 初始参数 |
| FW-DIFF-SIM-ARCHIVE | FW 差速早期仿真版 | ros1 | 15 / 2 | [fw_mini_diff_sim_archive.yaml](archive/fw_mini_diff_sim_archive.yaml) | 早期 RViz diff 配置 |
| UP-FW-OMNI-BASE-V1 | 上游全向 FW 初适配版 | upstream | 10 / 2 | [fw_mini_omni_base_v1.yaml](upstream/fw_mini_omni_base_v1.yaml) | 上游原核心，FW 车身/速度与 close_threshold 适配 |
| UP-FW-OMNI-TUNED-V2 | 上游全向 FW 调参版 | upstream | 10 / 2 | [fw_mini_omni_tuned_v2.yaml](upstream/fw_mini_omni_tuned_v2.yaml) | 上游原核心专用；不能等价加载于当前 ROS1 核心 |
| UP-SENTRY-OMNI-REFERENCE | 上游哨兵实车参考版 | upstream | 10 / 2 | [sentry_omni_reference.yaml](upstream/sentry_omni_reference.yaml) | 八边形哨兵、ref_speed 1.5 m/s，非 FW 模型 |
| EXP-REFERENCE-PORT | 上游参考逻辑移植实验 | reference_port | 10 / 2 | [fw_reference_port.yaml](experiments/reference_port/fw_reference_port.yaml) | 仅配合隔离 candidate_core；并非当前正式 ROS1 核心 |
| EXP-OMNI-DIFF-COSTS | 全向套用 diff 规划代价对照 | ros1 | 10 / 2 | [fw_omni_diff_costs.yaml](experiments/ros1/fw_omni_diff_costs.yaml) | omni 机器人段 + diff 规划代价，保留历史对照 |
| DIAG-OMNI-DIFF-UNITS | 只改模型名的单位诊断 | ros1 | 10 / 2 | [omni_diff_units_diagnostic.yaml](experiments/ros1/omni_diff_units_diagnostic.yaml) | 方向角界限仍为 0.4 rad，不能当作通用 FW 全向配置 |
| EXAMPLE-GENERIC-DIFF | 原仓库通用差速示例 | ros1 | 10 / 2 | [generic_diff.yaml](examples/generic_diff.yaml) | 1.6×2.0 m，非 FW-mini 几何 |
| SELFTEST-SMOKE | ROS1 冒烟自测配置 | ros1 | 10 / 2 | [smoke_selftest.yaml](examples/smoke_selftest.yaml) | 虚拟自测，非 FW 实车参数 |

当前 `config/fw_mini_omni.yaml` 对应 `FW-OMNI-LIGHT-TURN-V2`；`sim_complete.launch` 默认读取该入口。命名快照与入口不自动同步，运行其他快照时显式传入 config_file。

仅 `ros1/` 中的本地全向配置可直接配当前 ROS1 核心使用。`upstream/` 与 `reference_port` 记录用于对应的核心；当前正式 ROS1 核心不解析 projected_reference，不能把同一 YAML 在不同核心上当成等价试验。哨兵八边形和通用示例也不能使用 FW-mini 四边形模型。

本轮新配置仿真：

```bash
roslaunch neupan_cpp_ros1 sim_complete.launch \
  config_file:="$(rospack find neupan_cpp_ros1)/config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml"
```

共收录 136 个来源文件，按核心与参数内容去重为 91 套。完整编号和来源映射见 [index.yaml](index.yaml) 与 [all_profiles.csv](all_profiles.csv)。非主配置统一使用 TRIAL-核心-内容哈希编号；原试验名仍保留在 name/source_files 中，可搜索 rv16_rd0p5、q6_a012_w04 等旧名称。

扫描范围为项目主目录和本次性能/上游/急弯调参实验的配置；地图、场景 YAML、旧导出包副本单独保留，不重复作为规划器配置编号。

参数字段的中文名称与单位见 [CONFIG_PROFILES_CN.md](../../CONFIG_PROFILES_CN.md)。
