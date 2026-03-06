# 参数化机器人控制系统

通过调整物理参数来间接控制机器人，而非直接发送动作命令。

## 核心概念

### 传统控制 vs 参数化控制

**传统控制**:
```python
action = [0.5, -0.3, 0.8, ...]  # 直接指定关节扭矩
env.step(action)
```

**参数化控制**:
```python
controller.set_physics_param('motor_power_multiplier', 1.5)
# 控制器根据参数自动计算动作
result = controller.run_episode()
```

## 快速开始

### 基础用法

```python
from python_api.parametric_control import ParametricRobotController

# 创建控制器
controller = ParametricRobotController()

# 调整电机功率
controller.set_physics_param('motor_power_multiplier', 1.2)

# 运行并查看效果
result = controller.run_episode()
print(f"奖励: {result['total_reward']}")
```

### 可调参数

| 参数 | 默认值 | 范围 | 影响 |
|------|--------|------|------|
| `motor_power_multiplier` | 1.0 | 0.5-2.0 | 最大扭矩/速度 |
| `joint_stiffness` | 1.0 | 0.5-3.0 | 精度/震荡 |
| `joint_damping` | 0.5 | 0.1-1.0 | 稳定性/响应速度 |
| `friction` | 0.9 | 0.1-1.5 | 摩擦/能耗 |
| `mass_multiplier` | 1.0 | 0.5-1.5 | 惯性/所需力矩 |
| `gravity` | 9.81 | 0-20 | 重力环境 |

## 使用示例

### 示例 1: 实验不同配置

```python
# 高功率配置
controller.set_physics_param('motor_power_multiplier', 1.5)
result = controller.run_episode()

# 高精度配置
controller.set_physics_param('joint_stiffness', 2.0)
controller.set_physics_param('joint_damping', 0.7)
result = controller.run_episode()
```

### 示例 2: 参数扫描

```python
import numpy as np

# 扫描电机功率
for power in np.linspace(0.5, 2.0, 10):
    controller.set_physics_param('motor_power_multiplier', power)
    result = controller.run_episode()
    print(f"功率 {power:.2f}: 奖励 {result['total_reward']:.2f}")
```

### 示例 3: 自动优化

```python
# 定义搜索空间
param_ranges = {
    'motor_power_multiplier': (0.8, 1.5),
    'joint_stiffness': (0.5, 2.0),
    'joint_damping': (0.3, 0.8)
}

# 搜索最优配置
result = controller.find_optimal_params(param_ranges, n_trials=20)
print(f"最优参数: {result['best_params']}")
print(f"最高奖励: {result['best_reward']}")
```

### 示例 4: 交互式调整

```python
from python_api.parametric_control import InteractiveParameterTuner

tuner = InteractiveParameterTuner(controller)
tuner.interactive_tuning()

# 在交互模式中:
# > set motor_power_multiplier 1.3
# > test
# > show
# > quit
```

## 完整演示

```bash
# 运行所有示例
python examples/parametric_control_demo.py

# 运行特定示例
python examples/parametric_control_demo.py --demo 1  # 基础控制
python examples/parametric_control_demo.py --demo 2  # 参数扫描
python examples/parametric_control_demo.py --demo 3  # 自动优化

# 交互模式
python examples/parametric_control_demo.py --interactive
```

## 参数影响分析

### 电机功率 (motor_power_multiplier)

- **增加**: 更强的驱动力，能爬坡但能耗高
- **减少**: 节能但可能无法完成任务

**建议范围**: 0.8 - 1.5

### 关节刚度 (joint_stiffness)

- **增加**: 更精确，但可能震荡
- **减少**: 更柔和，但精度降低

**建议范围**: 0.5 - 2.0

### 关节阻尼 (joint_damping)

- **增加**: 更稳定，但响应慢
- **减少**: 响应快，但可能震荡

**建议范围**: 0.3 - 0.8

### 质量倍数 (mass_multiplier)

- **增加**: 更稳定，但需要更大力矩
- **减少**: 更灵活，但稳定性降低

**建议范围**: 0.7 - 1.3

## 实际应用

### 机器人设计验证

```python
# 测试轻量化设计
controller.set_physics_param('mass_multiplier', 0.7)
controller.set_physics_param('motor_power_multiplier', 0.9)
result = controller.run_episode()
```

### 环境适应

```python
# 模拟月球重力
controller.set_physics_param('gravity', 1.62)

# 模拟高摩擦地面
controller.set_physics_param('friction', 1.3)
```

### 性能优化

```python
# 自动找到最优配置
optimal = controller.find_optimal_params({
    'motor_power_multiplier': (0.8, 1.5),
    'joint_stiffness': (0.5, 2.0)
}, n_trials=30)
```

## 与零件库集成

```python
from python_api.custom_parts import CustomMotor, CustomJoint

# 创建定制电机
motor = CustomMotor({'power': 750, 'gear_ratio': 80})

# 将电机参数映射到控制器
power_multiplier = motor.params['power'] / 500.0  # 归一化
controller.set_physics_param('motor_power_multiplier', power_multiplier)

# 创建定制关节
joint = CustomJoint({'stiffness': 6000})

# 映射刚度
stiffness_multiplier = joint.params['stiffness'] / 5000.0
controller.set_physics_param('joint_stiffness', stiffness_multiplier)
```

## 常见问题

**Q: 如何选择初始参数？**
A: 从默认值开始，逐个参数调整并观察影响。

**Q: 参数优化需要多久？**
A: 10-20次试验通常足够，每次试验约5-10秒。

**Q: 可以保存最优配置吗？**
A: 可以，使用 `result['best_params']` 保存为JSON。

**Q: 参数变化会立即生效吗？**
A: 是的，下一次 `run_episode()` 时就会使用新参数。

## API 参考

### ParametricRobotController

```python
controller = ParametricRobotController(env_id='AGI-Walker/Walker2D-v0')

# 设置参数
controller.set_physics_param(param_name, value)

# 运行回合
result = controller.run_episode(max_steps=1000, render=False)

# 自动优化
optimal = controller.find_optimal_params(param_ranges, n_trials=10)
```

### InteractiveParameterTuner

```python
tuner = InteractiveParameterTuner(controller)

# 显示当前参数
tuner.show_current_params()

# 进入交互模式
tuner.interactive_tuning()
```

## 下一步

- [ ] 扩展到四足机器人
- [ ] 添加实时可视化
- [ ] 支持多目标优化
- [ ] 集成强化学习

---

**优势总结**:
- ✅ 直观：调整物理参数而非抽象动作
- ✅ 真实：参数对应实际零件属性
- ✅ 可解释：清楚影响机制
- ✅ 高效：自动搜索最优配置
