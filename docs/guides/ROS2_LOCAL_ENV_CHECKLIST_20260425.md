# ROS2 Local Env Checklist (2026-04-25)

这份清单现在记录的是：**本机已经如何满足 `tests/test_ros2_bridge_smoke.py` 的运行前提。**

## 当前机器状态

本机当前已经可以导入以下 ROS2 Python 模块：

- `rclpy`
- `tf2_ros`
- `sensor_msgs.msg`
- `geometry_msgs.msg`
- `std_srvs.srv`

因此当前 `ROS2 bridge smoke` 的状态是 **passed**。

## 当前通过方式

当前通过依赖的是这套组合：

- ROS2 前缀：`C:\dev\ros2_humble`
- ROS2 Python：`C:\dev\ros2_humble\.pixi\envs\default\python.exe`
- 运行前先执行：
  - `C:\dev\ros2_humble\local_setup.bat`

## 通过前提

要让专项 smoke 通过，需要同时满足：

- `local_setup.bat`
- ROS2 Pixi Python
- 仓库根目录在 `PYTHONPATH`
- ROS2 Pixi 环境里补齐仓库测试依赖

## 结果产物

专项报告会写到：

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

## 当前结论

截至 `2026-04-25`，本机状态是：

- 最小本地部署：已通过
- distributed 本地链路：已通过
- Godot headless smoke：已通过
- ROS2 bridge smoke：已通过

所以本机现在已经达到 **本地专项能力全绿**。
