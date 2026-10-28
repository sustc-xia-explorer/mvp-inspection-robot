# 工控机V2与历史/上游对照证据

完整说明见 [对照报告](../../../IPC_OMNI_COMPARISON_20261028_CN.md)。

- raw/：用户提供的最新V2三组工控机目录和此前FULL四组ROS会话、一组核心数据。
- developer_comparison/：同开发机的LIGHT V1、LIGHT TURN V2、上游原核心调参版。该目录不是工控机记录。
- audit/：72,770条工控机CSV的独立复核、输入散列和三张无路网格证明。
- plots/：工控机核心对比、最新ROS走廊/急弯图、开发机上游对照。
- algorithm_sources.json / algorithm_diffs/：本地与上游冻结提交的源码散列、差异。
- upstream_*：上游预测参数、来源、到达后停车和连续目标审计证据。

复核范围是静态地图理想底盘仿真，未包含真实档位7执行适配。
