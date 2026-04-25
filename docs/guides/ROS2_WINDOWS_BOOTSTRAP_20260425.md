# ROS2 Windows Bootstrap (2026-04-25)

这份说明现在记录的是：**这台 Windows 机器怎样被补齐到可以跑通 `ROS2 bridge smoke`。**

## 当前机器最终状态

当前已经完成：

- 官方 ROS2 Humble Windows 包已安装到 `C:\dev\ros2_humble`
- `pixi` 已安装
- `pixi install` 已执行
- `preinstall_setup_windows.py` 已执行
- `ROS2 bridge smoke` 已通过

## 当前可用执行方式

当前可用的专项执行方式不是直接用系统 Python，而是：

```powershell
call C:\dev\ros2_humble\local_setup.bat
set PYTHONPATH=C:\temp\agiwalker_repo;%PYTHONPATH%
set AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
C:\dev\ros2_humble\.pixi\envs\default\python.exe -m pytest tests\test_ros2_bridge_smoke.py -q -m "integration and live"
```

## 最终结果

报告位置：

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

## 如果你要继续深入

ROS2 workspace / launch / topic / service 的仓库内入口在：

- `docs/ros2/ROS2_QUICK_START.md`

## 当前最务实的判断

如果你现在只是继续本地开发、演示、跑主线功能：

- 当前状态已经够用

如果你要复现本机专项全绿：

- 直接按本页的 ROS2 环境执行方式走即可
