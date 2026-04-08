# Parametric Control

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里的参数化控制思路。这里的“参数化”主要指通过一组可调参数塑造控制行为，而不是固定写死一条控制曲线。

## 1. 当前相关模块

核心文件：

- `agi_walker/core/controllers/pid_controller.py`
- `agi_walker/core/controllers/pid_tuner.py`
- `agi_walker/core/controllers/physics_tuner.py`
- `agi_walker/skills/parameter_optimizer.py`

它们分别覆盖：

- PID 控制
- PID 参数搜索
- 物理参数扫描
- 质量分布和参数优化 skill

## 2. 控制参数常见维度

当前仓库里最常见的参数包括：

- `kp`
- `ki`
- `kd`
- `motor_power_multiplier`
- `joint_stiffness`
- `joint_damping`
- `friction`
- `gravity`

这些参数并不都属于同一层级：

- PID 是控制器层参数
- stiffness / damping 更接近执行器或物理侧参数
- friction / gravity 更接近环境参数

## 3. 当前最直接可复用的控制器

`pid_controller.py` 提供：

- `PIDController`
- `BalanceController`

`BalanceController` 当前使用两个 PID 回路：

- roll PID
- pitch PID

并输出：

- `hip_left`
- `hip_right`

## 4. 调参与搜索

### PID 调参

`pid_tuner.py` 提供：

- 单组参数测试
- grid search
- adaptive search

注意：

- 它依赖真实 Godot TCP 仿真器
- 不属于默认无依赖本地路径

### 物理参数扫描

`physics_tuner.py` 更偏向：

- 稳定性测试
- 电机响应测试
- 参数扫描框架

同样依赖仿真器连接。

## 5. Skills 与 parametric control 的关系

如果目标是模型结构或质量分布优化，更应该看：

- `parameter-optimizer` skill

如果目标是姿态控制与回路调参，更应该看：

- `PIDController`
- `PIDTuner`

## 6. 推荐工作流

一个现实的顺序是：

1. 先建立稳定 baseline
2. 再只调一类参数
3. 保持输出目录和结果记录隔离
4. 最后再做多参数联合搜索

不要在还没验证通信链路时就同时调 PID、摩擦、质量和环境。

## 7. 验证建议

轻量层：

```bash
python -m pytest tests/test_pid_balance.py -q
```

需要说明：

- 该测试是 integration 标记
- 若 Godot 未运行，测试会跳过

## 8. 当前边界

- 不是所有参数都已经统一成可视化配置面
- 部分调参工具仍带历史导入方式
- 很多调参路径依赖真实仿真器，而不是纯单元测试

## 结论

AGI-Walker 当前的参数化控制能力主要集中在 PID 回路、参数搜索和物理侧扫描。先明确你在调“控制器参数”还是“环境参数”，会比盲目做大规模搜索更有效。
