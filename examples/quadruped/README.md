# 四足机器�?(Quadruped Robot)

12 DoF 四足机器人，支持多种步态�?

## 特�?

- **12 自由�?* - 4条腿，每�?个关�?
- **多种步�?* - Trot (小跑), Gallop (疾驰), Walk (行走)
- **地形适应** - 可在不同地形上行�?
- **完整训练流程** - 从零开始到部署

## 快速开�?

### 1. 创建环境

```python
import gymnasium as gym

# 注册四足环境
env = gym.make('AGI-Walker/Quadruped-v0')

obs, info = env.reset()
print(f"观测空间: {env.observation_space}")
print(f"动作空间: {env.action_space}")
```

### 2. 使用步态生成器

```python
from python_api.gait_generator import GaitController

controller = GaitController()

# Trot 步�?
controller.set_gait('trot')
joint_targets = controller.get_joint_targets(phase=0.0)

# Gallop 步�?
controller.set_gait('gallop')
joint_targets = controller.get_joint_targets(phase=0.0)
```

### 3. 训练模型

```bash
# 开始训�?
python examples/quadruped_training.py --train --timesteps 1000000

# 测试模型
python examples/quadruped_training.py --test models/quadruped/ppo_quadruped_final.zip
```

## 环境详情

### 观测空间 (34�?

- **身体状�?(9�?**:
  - 姿�?(roll, pitch, yaw)
  - 角速度 (x, y, z)
  - 线速度 (x, y, z)

- **关节状�?(24�?**:
  - 12个关节角�?
  - 12个关节角速度

- **目标 (1�?**:
  - 目标方向

### 动作空间 (12�?

12个关节的目标位置（归一化到 [-1, 1]�?
- FL: Hip, Thigh, Shin
- FR: Hip, Thigh, Shin
- RL: Hip, Thigh, Shin
- RR: Hip, Thigh, Shin

### 奖励函数

```python
reward = forward_velocity * 1.0      # 前进速度
       + energy * (-0.05)            # 能量惩罚
       + stability * 0.5             # 稳定�?
       + orientation * 0.3           # 姿�?
       + height_error * (-0.2)       # 高度
```

## 步态类�?

### Trot (小跑)
- 频率: 1.0 Hz
- 相位: 对角腿同�?(FL-RR, FR-RL)
- 适用: 中速稳定运�?

### Gallop (疾驰)
- 频率: 1.5 Hz
- 相位: 前后腿分别同�?
- 适用: 高速冲�?

### Walk (行走)
- 频率: 0.5 Hz
- 相位: 三足支撑
- 适用: 低速稳定，复杂地形

## 机器人规�?

| 参数 | �?|
|------|-----|
| 质量 | 12 kg |
| 身高 | 0.3 m |
| 腿长 | 0.3 m (大腿+小腿) |
| 最大速度 | ~2 m/s (Gallop) |
| 最大爬�?| 30° |

## 训练技�?

### 1. 课程学习

从简单到复杂:
1. 平地 Trot 步�?
2. 不平地形
3. Gallop 高速运�?

### 2. 奖励调整

- 初期：增�?`stability` 权重
- 中期：增�?`forward_velocity` 权重
- 后期：添加地形适应奖励

### 3. 超参�?

```python
# PPO 推荐配置
learning_rate = 3e-4
n_steps = 2048
batch_size = 64
ent_coef = 0.01  # 探索系数
```

## 性能基准

| 指标 | 目标 | 当前最�?|
|------|------|----------|
| 前进速度 | >1.0 m/s | - |
| 稳定�?| >95% | - |
| 能效 | <10 W | - |

## 常见问题

**Q: 为什么机器人摔倒？**
A: 增加稳定性奖励权重，减小学习率�?

**Q: 如何切换步态？**
A: 使用 `GaitController.set_gait(gait_name)`�?

**Q: 能否自定义步态？**
A: 可以，继�?`GaitGenerator` 类并实现 `get_foot_trajectory`�?

## 下一�?

- [ ] 添加障碍物回�?
- [ ] 多地形适应
- [ ] 动态步态切�?
- [ ] 真实机器人部�?

## 参�?

- [MIT Cheetah](https://biomimetics.mit.edu/)
- [Unitree Robotics](https://www.unitree.com/)
- [RL for Quadrupeds](https://arxiv.org/abs/1804.10332)
