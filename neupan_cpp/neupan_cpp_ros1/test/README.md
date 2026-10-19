# 本目录的测试版本

原有 smoke_test.py、closed_loop_test.py、path_overshoot_test.py、sentry_smoke_test.py、astar_test.py 是 ROS2/rclpy 参考脚本，不能直接使用 Noetic 运行。

ROS1 可执行的自检和场景仿真在 `../scripts/`：

```bash
rosrun neupan_cpp_ros1 smoke_test.py
rosrun neupan_cpp_ros1 simulation_test.py --cases all
```

独立模拟几何检查可直接运行 `python3 test_simulation_geometry.py`。完整分层验证、模型对照及当前未通过项见 [仿真验证说明](../../SIMULATION_VALIDATION_CN.md)。

交互地图的全车体路径检查、扫描与非法目标检查运行 `python3 test_rviz_world.py`；ROS1 RViz 点选目标的运行方法见 [交互仿真](../../RVIZ_SIMULATION_CN.md)。
