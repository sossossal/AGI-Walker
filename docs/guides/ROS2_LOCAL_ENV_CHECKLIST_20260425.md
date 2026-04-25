# ROS2 Local Env Checklist (2026-04-25)

这份清单只解决一件事：让本机满足 `tests/test_ros2_bridge_smoke.py` 的运行前提。

## 当前机器确认缺失

本机刚刚通过 `runtime_probe.py` 实测缺少以下 ROS2 Python 模块：

- `rclpy`
- `tf2_ros`
- `sensor_msgs.msg`
- `geometry_msgs.msg`
- `std_srvs.srv`

因此当前 `ROS2 bridge smoke` 的状态是 **skip**，不是代码失败。

## 最短补齐目标

你只需要让当前终端里的 Python 能导入下面 5 个模块：

```python
import rclpy
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
```

## 推荐顺序

### 1. 安装并准备 ROS2 Humble

按仓库现有专项文档执行：

- `docs/ros2/ROS2_QUICK_START.md`

重点关注：

- ROS2 workspace：`hardware/ros2_ws`
- bridge 实现：`hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py`

### 2. 进入带 ROS2 环境的终端

确保你当前运行 pytest 的终端已经加载 ROS2 环境。核心要求不是“装过 ROS2”，而是 **当前 shell 可以导入 ROS2 Python 包**。

### 3. 先跑运行时探针

推荐先在当前终端执行：

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
- `missing_modules` 为空

### 4. 再跑专项 smoke

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

### 5. 看产物

专项报告会写到：

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

## 当前结论

截至 `2026-04-25`，本机状态是：

- 最小本地部署：已通过
- distributed 本地链路：已通过
- Godot headless smoke：已通过
- ROS2 bridge smoke：因 ROS2 Python runtime 缺失而跳过

所以离“本地专项能力全绿”只差 **ROS2 本机运行时** 这一项。
