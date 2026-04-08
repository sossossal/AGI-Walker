# Physics Environment Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库中的物理环境与随机化能力。这里既包括 Python 侧的训练环境，也包括 Godot 侧的环境控制脚本。

## 1. Python 侧环境随机化

核心文件：

- `agi_walker/core/api/godot_robot_env/gym_env.py`
- `agi_walker/core/api/godot_robot_env/domain_randomizer.py`

`GodotRobotEnv` 使用 `RandomizerConfig` 描述环境随机化参数，当前主要字段包括：

- `mass_range`
- `friction_range`
- `motor_strength_range`
- `motor_lag_range`
- `sensor_noise_std`
- `enable_randomization`

这些参数会在 `reset()` 时转成当前 episode 的 `sim_params`。

## 2. 当前 reward 与终止逻辑

`GodotRobotEnv` 当前的 reward 由几部分组成：

- 生存奖励
- 姿态平衡惩罚
- 动作幅度惩罚
- 高度保持惩罚
- 双脚接触奖励

终止条件主要是：

- `roll` 或 `pitch` 超限
- 躯干高度过低

这意味着当前环境偏向平衡与站立稳定，而不是复杂 locomotion benchmark。

## 3. Godot 侧环境脚本

当前仓库有一批 Godot 环境脚本：

- `godot_project/scripts/environment/environment_controller.gd`
- `godot_project/scripts/environment/procedural_terrain.gd`
- `godot_project/scripts/environment/ground_material.gd`
- `godot_project/scripts/environment/ground_material_library.gd`
- `godot_project/scripts/environment/ground_tilt_controller.gd`
- `godot_project/scripts/environment/environment_ui.gd`

这些脚本说明环境参数控制主要仍在 Godot 侧完成。

## 4. 环境预设

`environment_controller.gd` 当前内置环境预设包括：

- `earth`
- `moon`
- `mars`
- `jupiter`
- `custom`

主要环境参数有：

- gravity
- air_density
- temperature
- wind_velocity
- ground_friction

## 5. 测试场景

当前 Godot 测试环境场景：

- `godot_project/scenes/test_environment.tscn`

它会挂载：

- `test_environment.gd`
- `EnvironmentController`
- `GroundMaterialLibrary`

这说明环境能力已经有 Godot 场景载体，但仍不属于默认 CLI 主线路。

## 6. 物理参数来源差异

当前仓库里与“物理参数”相关的名字并不完全统一，常见有三套：

- `mass_scale` / `friction_scale` / `motor_strength`
- `friction_coefficient` / `damping_ratio` / `contact_stiffness`
- `motor_power_multiplier` / `joint_stiffness` / `joint_damping`

它们分别来自：

- 仿真 episode 随机化
- Sim2Real 校正
- UI / API 调参与控制参数

不要把这三套字段直接视为一一等价。

## 7. 什么时候使用环境随机化

适合：

- RL 训练鲁棒性增强
- Sim2Real gap 缩小
- 扰动测试

不适合：

- 刚开始排查基本通信问题
- 还没建立稳定 baseline 的控制实验

## 8. 建议验证顺序

1. 先关闭随机化，确认仿真主链可用
2. 再开启 `enable_randomization`
3. 再逐步扩大质量、摩擦和电机范围

## 结论

当前物理环境能力已经覆盖了随机化、环境预设和 Godot 侧环境控制，但它更适合作为增强层，而不是最先调试的基础层。先守住稳定 baseline，再加环境复杂度。
