# 本目录的测试版本

原有 smoke_test.py、closed_loop_test.py、path_overshoot_test.py、sentry_smoke_test.py、astar_test.py 是 ROS2/rclpy 参考脚本，不能直接使用 Noetic 运行。

ROS1 可执行的自检和场景仿真在 `../scripts/`：

```bash
rosrun neupan_cpp_ros1 smoke_test.py
rosrun neupan_cpp_ros1 simulation_test.py --cases all
```

独立模拟几何检查可直接运行 `python3 test_simulation_geometry.py`。完整分层验证、模型对照及当前未通过项见 [仿真验证说明](../../SIMULATION_VALIDATION_CN.md)。

交互地图的全车体路径检查、扫描与非法目标检查运行 `python3 test_rviz_world.py`；ROS1 RViz 点选目标的运行方法见 [交互仿真](../../RVIZ_SIMULATION_CN.md)。

## 包内 omni / diff 移植验证

新增 `core/test_omni.cpp`、`data/omni_python_reference.json`、`model_closed_loop.py`、`ros1_model_test.py` 和 `test_planar_kinematics.py` 用于本次 ROS1 全向移植。编译和运行命令见 [包说明](../README.md)。旧 ROS2 参考文件保持原样。

走廊平滑、周期角度和连续选点的复测见 [SMOOTH_CORRIDOR_CN.md](../SMOOTH_CORRIDOR_CN.md)，入口为 `tune_omni_corridor.py` 与 `ros1_smooth_corridor_test.py`。
