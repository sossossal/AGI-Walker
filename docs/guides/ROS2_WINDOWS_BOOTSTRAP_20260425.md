# ROS2 Windows Bootstrap (2026-04-25)

这份说明只解决一个现实问题：**当前这台 Windows 机器还没有可见的 ROS2 运行时，怎样用最短路径补齐到能跑 `ROS2 bridge smoke`。**

## 当前机器诊断结论

刚刚已经确认：

- `where ros2`：找不到
- 当前终端没有 `ROS_* / AMENT_* / COLCON_* / RMW_*` 环境变量
- 常见安装目录下没有可见 ROS2 安装
- 当前 Python 也无法导入：
  - `rclpy`
  - `tf2_ros`
  - `sensor_msgs.msg`
  - `geometry_msgs.msg`
  - `std_srvs.srv`

所以当前问题不是“忘了 source 环境”，而是 **本机还没有可用的 ROS2 Python runtime**。

## 最短目标

目标不是先把整套 ROS2 工具链研究透，而是先满足这一条专项测试：

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

为此，你当前终端至少需要能导入：

```python
import rclpy
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
```

## 推荐顺序

### 1. 先安装 ROS2 Humble（Windows）

优先以 ROS2 Humble 为目标，因为仓库专项测试就是按 `humble smoke` 命名的。

安装完成后，至少要具备：

- `ros2.exe`
- Python 包：
  - `rclpy`
  - `tf2_ros`
  - `sensor_msgs`
  - `geometry_msgs`
  - `std_srvs`

### 2. 用 ROS2 终端打开仓库

不要在“普通 PowerShell”里直接重跑测试。  
应该先进入已经加载 ROS2 环境的终端，再进入仓库目录。

### 3. 先跑运行时探针

进入仓库后，先执行：

```powershell
@'
import importlib.util, json, pathlib
path = pathlib.Path(r"hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/runtime_probe.py")
spec = importlib.util.spec_from_file_location("runtime_probe", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
print(json.dumps(module.probe_ros2_python_runtime(), ensure_ascii=False, indent=2))
'@ | python -
```

通过标准：

- `"available": true`
- `missing_modules: []`

### 4. 再跑仓库专项 smoke

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

### 5. 最后看报告

报告位置：

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

## 如果你要继续深入

ROS2 workspace / launch / topic / service 的仓库内入口在：

- `docs/ros2/ROS2_QUICK_START.md`

## 当前最务实的判断

如果你现在只是继续本地开发、演示、跑主线功能：

- 当前状态已经够用

如果你要把“本机专项能力全绿”补齐：

- 下一步唯一高价值动作就是安装并接通 ROS2 Humble 运行时
