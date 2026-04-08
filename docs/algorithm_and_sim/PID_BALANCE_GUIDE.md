# PID Balance Guide

更新日期：`2026-04-08`

本页说明如何使用 AGI-Walker 当前的 PID 平衡控制模块。

## 1. 当前核心类

文件：

- `agi_walker/core/controllers/pid_controller.py`

主要类：

- `PIDController`
- `BalanceController`

`PIDController` 负责单回路控制，`BalanceController` 使用 roll / pitch 两个 PID 回路生成双髋关节修正量。

## 2. `PIDController` 提供什么

当前能力包括：

- `compute(setpoint, measured_value)`
- `compute_from_error(error, dt)`
- 输出限幅
- 积分限幅
- 动态 `set_tunings`
- `reset()`

这已经足够做独立的 Python 侧控制实验。

## 3. `BalanceController` 的输入输出

输入假设为：

- `sensor_data["sensors"]["imu"]["orient"]`

输出格式为：

```python
{
    "motors": {
        "hip_left": ...,
        "hip_right": ...,
    }
}
```

这与当前控制栈的动作字典风格保持一致。

## 4. 最小示例

```python
from agi_walker.core.controllers.pid_controller import BalanceController

controller = BalanceController(
    roll_pid_params=(8.0, 0.5, 3.0),
    pitch_pid_params=(8.0, 0.5, 3.0),
)

sensor = {
    "sensors": {
        "imu": {"orient": [3.0, -2.0, 0.0]},
    }
}

command = controller.compute_balance(sensor, dt=0.01)
print(command)
```

## 5. 调参工具

文件：

- `agi_walker/core/controllers/pid_tuner.py`

当前支持：

- 单组参数测试
- `grid_search`
- `adaptive_search`

但要注意：

- 这些方法依赖 `GodotClient`
- 需要真实仿真器连接

## 6. 当前测试状态

`tests/test_pid_balance.py` 是 integration 测试。它会在下面场景跳过：

- `GodotClient` 不可用
- 无法连接到仿真器

因此它更适合作为专项验证，不适合文档或日常小改动后的默认测试。

## 7. 实践建议

推荐顺序：

1. 先在纯 Python 层验证 PID 数值行为
2. 再进入 Godot 集成测试
3. 最后再使用 PIDTuner 做批量搜索

## 8. 常见问题

### 输出抖动

优先检查：

- `kd` 是否过大
- `dt` 是否不稳定
- 积分是否累积过头

### 很快饱和

优先检查：

- 输出限幅
- 积分限幅
- 输入姿态单位是否一致

### Godot 中无效果

优先检查：

- TCP 链路是否连通
- 传感器数据结构是否符合控制器假设
- 电机命令是否真的发送到仿真器

## 结论

当前 PID 平衡栈已经足够做小规模控制实验和集成验证。先把 `PIDController` 和 `BalanceController` 的局部行为跑通，再上 Godot 和自动调参，会更高效。
