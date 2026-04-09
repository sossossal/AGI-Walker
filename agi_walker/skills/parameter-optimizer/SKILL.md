---
name: parameter-optimizer
description: "自动优化机器人质量分布与 PID 控制参数，支持梯度法、遗传算法和批量调优。"
category: 优化
emoji: "⚙️"
inputs:
  robot_config:
    type: file_path
    description: 机器人配置文件路径，或等价的配置对象
  target_com_height:
    type: number
    description: 目标重心高度
    required: false
  joint_name:
    type: string
    description: 需要调优的关节名称
    required: false
  method:
    type: string
    description: 优化方法，如 gradient、genetic、ziegler_nichols
    required: false
outputs:
  optimization_result:
    type: dict
    description: 质量分布优化结果
  pid_gains:
    type: dict
    description: PID 调优结果
metadata:
  agi_walker:
    requires:
      python_modules:
        - scipy
        - numpy
---

# Parameter Optimizer Skill

用于优化机器人参数，重点覆盖质量分布和 PID 控制器增益。

当前 skill 的真实入口位于 `__init__.py`，核心能力包括：

- `optimize_mass_distribution(...)`
- `tune_pid_controller(...)`
- `batch_optimize_pid(...)`

## 适用场景

- 调整各部件质量以逼近目标重心高度
- 为单个关节自动生成 PID 初始增益
- 批量调优多个关节的控制参数

## 质量分布优化

质量分布优化由 `MassDistributionOptimizer` 实现。

- 输入：机器人配置、目标重心高度、迭代次数、优化方法
- 输出：质量分布、重心位置、重心误差、是否成功

支持的方法：

- `gradient`: 基于 `scipy.optimize.minimize`
- `genetic`: 基于 `scipy.optimize.differential_evolution`

示例：

```python
from agi_walker.skills.parameter_optimizer import optimize_mass_distribution

result = optimize_mass_distribution(
    "configs/my_robot.json",
    target_com_height=0.25,
    max_iterations=100,
    method="gradient",
)

print(result.mass_distribution)
print(result.com_error)
```

## PID 调优

PID 调优由 `PIDTuner` 实现。

支持的方法：

- `ziegler_nichols`: 快速生成工程可用的初始增益
- `genetic`: 通过遗传算法搜索更优参数

示例：

```python
from agi_walker.skills.parameter_optimizer import tune_pid_controller

gains = tune_pid_controller(
    "configs/my_robot.json",
    joint_name="hip_flex",
    method="ziegler_nichols",
)

print(gains.kp, gains.ki, gains.kd)
```

## 批量调优

可通过 `batch_optimize_pid(...)` 为多个关节一次性生成参数：

```python
from agi_walker.skills.parameter_optimizer import batch_optimize_pid

results = batch_optimize_pid(
    "configs/my_robot.json",
    joint_names=["hip_left", "hip_right", "knee_left", "knee_right"],
    method="ziegler_nichols",
)
```
