---
name: parameter-optimizer
description: "自动优化机器人参数(质量分布/关节限位/PID增益),使用遗传算法或梯度方法。适用于:(1)优化重心位置以提高稳定性 (2)调优控制器参数 (3)多目标优化 (4)自动化参数搜索"
metadata:
  agi_walker:
    emoji: "⚙️"
    category: "优化"
    requires:
      python_modules: ["scipy", "numpy"]
---

# Parameter Optimizer Skill

自动优化机器人参数,提升性能和稳定性。

## 快速开始

### 优化重心位置

```python
from agi_walker.skills.parameter_optimizer import optimize_mass_distribution

result = optimize_mass_distribution(
    robot_config="configs/my_robot.json",
    target_com_height=0.25,  # 期望重心高度(米)
    max_iterations=100
)

print(f"优化后质量分布: {result.mass_distribution}")
print(f"重心偏移: {result.com_error:.4f} m")
```

### PID参数自动调优

```python
from agi_walker.skills.parameter_optimizer import tune_pid_controller

gains = tune_pid_controller(
    robot_config="configs/my_robot.json",
    joint_name="hip_flex",
    method="ziegler_nichols",  # 或 "genetic"
    simulation_steps=1000
)

print(f"优化PID增益: Kp={gains.kp}, Ki={gains.ki}, Kd={gains.kd}")
```

## 优化方法

### 1. 质量分布优化

**目标**: 调整各部件质量,使重心达到期望位置。

**算法**: 
- 梯度下降法 (快速,局部最优)
- 遗传算法 (慢速,全局最优)

**使用场景**:
- 提高双足机器人稳定性 (降低重心)
- 平衡四足机器人重量分配
- 满足硬件约束 (总质量限制)

### 2. PID增益调优

**支持方法**:

#### Ziegler-Nichols法
经典工程方法,快速获得初始增益。

```python
gains = tune_pid_controller(
    robot_config="configs/robot.json",
    joint_name="knee",
    method="ziegler_nichols"
)
```

#### 遗传算法
搜索全局最优增益,适合复杂系统。

```python
gains = tune_pid_controller(
    robot_config="configs/robot.json",
    joint_name="knee",
    method="genetic",
    population_size=50,
    generations=100,
    fitness_metric="ise"  # ISE/IAE/ITAE
)
```

### 3. 多目标优化

同时优化多个指标 (速度/稳定性/能耗)。

```python
from agi_walker.skills.parameter_optimizer import multi_objective_optimize

result = multi_objective_optimize(
    robot_config="configs/robot.json",
    objectives=[
        {"name": "speed", "weight": 0.5, "target": "maximize"},
        {"name": "stability", "weight": 0.3, "target": "maximize"},
        {"name": "energy", "weight": 0.2, "target": "minimize"}
    ],
    method="nsga2"  # NSGA-II算法
)

# 获取帕累托前沿
pareto_solutions = result.pareto_front
```

## 命令行工具

### 批量优化

```bash
python -m agi_walker.skills.parameter_optimizer.batch_optimize \
    --config configs/robot.json \
    --optimize mass,pid,damping \
    --output results/optimized.json
```

### 参数扫描

```bash
python -m agi_walker.skills.parameter_optimizer.param_sweep \
    --config configs/robot.json \
    --param leg_length \
    --range 0.2,0.5,0.05 \
    --metric stability
```

## 配置文件

创建优化配置 `optimization_config.yaml`:

```yaml
robot_config: "configs/my_robot.json"

optimizations:
  - type: mass_distribution
    target_com_height: 0.25
    constraints:
      total_mass: [5.0, 10.0]  # 最小,最大
    
  - type: pid_tuning
    joints: ["hip_flex", "knee_flex", "ankle_flex"]
    method: genetic
    
  - type: joint_damping
    range: [0.1, 1.0]
    optimize_for: stability
```

运行:
```bash
python -m agi_walker.skills.parameter_optimizer.run_config \
    --config optimization_config.yaml
```

## API参考

详见: `references/api.md`

## 常见问题

**Q: 优化需要多长时间?**
A: 
- 梯度法: 通常 <1分钟
- 遗传算法: 5-30分钟 (取决于种群和代数)
- 多目标优化: 10-60分钟

**Q: 如何选择优化方法?**
A:
- 快速原型: 使用梯度法
- 精确结果: 使用遗传算法
- 复杂系统: 使用多目标优化

**Q: 优化结果不理想怎么办?**
A:
1. 增加迭代次数
2. 调整约束范围
3. 更换优化算法
4. 检查仿真精度

## 示例: 完整优化流程

```python
from agi_walker.skills.robot_modeling import load_template
from agi_walker.skills.parameter_optimizer import (
    optimize_mass_distribution,
    tune_pid_controller
)

# 1. 加载机器人
robot = load_template("biped_basic")

# 2. 优化质量分布
mass_result = optimize_mass_distribution(
    robot,
    target_com_height=0.22,
    max_iterations=200
)

# 应用优化结果
for part_id, new_mass in mass_result.mass_distribution.items():
    robot.update_part_mass(part_id, new_mass)

# 3. 调优PID
for joint in ["hip_flex_left", "hip_flex_right", "knee_left", "knee_right"]:
    gains = tune_pid_controller(
        robot,
        joint_name=joint,
        method="ziegler_nichols"
    )
    robot.set_pid_gains(joint, gains.kp, gains.ki, gains.kd)

# 4. 保存优化后的配置
robot.save("configs/optimized_biped.json")
```

## 性能优化建议

1. **并行化**: 使用多进程加速参数扫描
2. **早停**: 设置收敛阈值避免过度优化
3. **缓存**: 复用相似配置的仿真结果
4. **精度权衡**: 减少仿真步数换取速度 (初期优化)

## 下一步

优化完成后,可以:

1. **仿真验证**: 使用 Godot 查看优化效果
2. **生成数据**: 使用 `simulation-runner` skill 生成训练数据
3. **格式转换**: 使用 `urdf-generator` skill 导出到其他平台

---

**相关 Skills**: `robot-modeling`, `simulation-runner`, `urdf-generator`
