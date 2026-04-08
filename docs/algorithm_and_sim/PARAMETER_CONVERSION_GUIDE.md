# Parameter Conversion Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库中几套常见参数命名之间的关系。它的目标不是给出一个严格数学变换，而是帮助你在 workflow、Web、仿真和 Sim2Real 之间减少混淆。

## 1. 为什么需要这页

当前仓库里至少存在三套常用参数命名：

### 控制/UI 参数

常见于 Web、GUI 或控制文案：

- `motor_power_multiplier`
- `joint_stiffness`
- `joint_damping`
- `friction`
- `gravity`

### 仿真 episode 参数

常见于 `GodotRobotEnv` 和随机化：

- `mass_scale`
- `friction_scale`
- `motor_strength`
- `motor_lag`

### Sim2Real / 物理校正参数

常见于 `sim2real_gap.py`：

- `friction_coefficient`
- `damping_ratio`
- `contact_stiffness`
- `joint_friction`
- `gravity`
- `mass_scale`

## 2. 近似映射关系

下面是当前项目里最实用的近似映射：

- `motor_power_multiplier` 约等于 `motor_strength`
- `friction` 近似对应 `friction_scale` 或 `friction_coefficient`
- `joint_damping` 近似对应 `damping_ratio`
- `joint_stiffness` 近似对应 `contact_stiffness`
- `mass_multiplier` 或结构质量调整，近似对应 `mass_scale`

需要注意：

- 这些是工程近似，不是严格同义词
- 不同模块对数值范围和单位的假设不完全一致

## 3. 当前哪些地方会用到这些参数

### Web / API

`parts_sim2real_api.py` 和部分历史 GUI 逻辑中常出现：

- `motor_power_multiplier`
- `joint_stiffness`
- `joint_damping`

### Gym 环境

`GodotRobotEnv` 会在 `reset()` 中发送：

- `physics_config`
- `sim_params`

其中 `sim_params` 当前包含：

- `mass_scale`
- `friction_scale`
- `motor_strength`
- `motor_lag`

### Sim2Real 校正

`Sim2RealGapEstimator` 当前输出和维护的是：

- `PhysicsParams`

## 4. 一个务实的转换策略

推荐做法不是一次性做全自动映射，而是分两步：

1. 先选定当前任务的主参数空间
2. 只对必要字段做显式转换

例如：

- 做 UI 调参时，以 `motor_power_multiplier / joint_stiffness / joint_damping` 为主
- 做仿真随机化时，以 `mass_scale / friction_scale / motor_strength` 为主
- 做 Sim2Real 校正时，以 `PhysicsParams` 为主

## 5. 一个简单参考表

```text
UI / control               Simulation             Sim2Real
--------------------------------------------------------------
motor_power_multiplier  -> motor_strength      -> (no direct exact field)
friction                -> friction_scale      -> friction_coefficient
joint_damping           -> (no exact field)    -> damping_ratio
joint_stiffness         -> (scene/contact cfg) -> contact_stiffness
mass_multiplier         -> mass_scale          -> mass_scale
gravity                 -> gravity             -> gravity
```

## 6. 转换时最常见的错误

- 直接把比例因子当作绝对物理量
- 把 `joint_stiffness` 当成 `contact_stiffness`
- 忽略 `motor_lag`
- 在不同模块间沿用同名字段，却不做范围裁剪

## 7. 建议

- 先在文档或代码里明确“这一步使用哪一套参数空间”
- 跨模块传值时做显式命名转换
- 对范围做 clamp
- 需要长期复用时，把转换逻辑收敛成独立函数，而不是散落在脚本里

## 结论

AGI-Walker 当前没有统一的“全局参数标准名”。最务实的方式是承认参数空间分层存在，并在 Web、仿真和 Sim2Real 之间做显式、局部、可审查的转换。
