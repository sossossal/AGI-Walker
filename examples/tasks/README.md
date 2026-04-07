# AGI-Walker 标准任务�?

本目录包�?AGI-Walker 的标�?RL 任务,每个任务都提�?
- �?Gymnasium 环境
- �?训练脚本
- �?预训练模�?(部分)
- �?性能 Baseline

---

## 任务列表

### 1. Locomotion (运动)

#### �?楼梯攀�?(Stair Climbing)
**路径**: `stair_climbing/`  
**难度**: ⭐⭐�? 
**目标**: 从平地爬�?5 级楼�? 
**机器�?*: 四足/双足  
**Baseline**: 成功�?85%, 平均奖励 12.5

**快速开�?*:
```bash
python examples/tasks/stair_climbing/env.py
```

#### 🔄 崎岖地形 (Rough Terrain) - 开发中
**路径**: `rough_terrain/`  
**难度**: ⭐⭐⭐⭐  
**目标**: 在随机生成的崎岖地形上行�?10m

#### 🔄 斜坡行走 (Slope Walking) - 规划�?
**难度**: ⭐⭐

---

### 2. Manipulation (操作)

#### 🔄 物体抓取 (Object Grasping) - 开发中
**路径**: `object_grasping/`  
**难度**: ⭐⭐⭐⭐  
**目标**: 抓取桌面上的随机物体  
**机器�?*: 机械�?

#### 🔄 门把手操�?(Door Opening) - 规划�?
**难度**: ⭐⭐⭐⭐�?

#### 🔄 堆叠积木 (Block Stacking) - 规划�?
**难度**: ⭐⭐⭐⭐

---

### 3. Navigation (导航)

#### 🔄 避障导航 (Obstacle Avoidance) - 开发中
**路径**: `obstacle_avoidance/`  
**难度**: ⭐⭐�? 
**目标**: 在动态障碍物环境中到达目标点

#### 🔄 动态环境导�?(Dynamic Navigation) - 规划�?
**难度**: ⭐⭐⭐⭐

---

### 4. Multi-Agent (多智能体)

#### 🔄 协作搬运 (Collaborative Carrying) - 规划�?
**难度**: ⭐⭐⭐⭐�? 
**目标**: 两个机器人协作搬运重�?

#### 🔄 编队行走 (Formation Walking) - 规划�?
**难度**: ⭐⭐�?

---

## 使用指南

### 1. 运行任务

```python
import gymnasium as gym

# 创建环境
env = gym.make('StairClimbing-v0')

# 重置
obs, info = env.reset()

# 运行
for _ in range(1000):
    action = policy.get_action(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        break
```

### 2. 训练模型

```bash
# 使用 PPO 训练
python examples/tasks/stair_climbing/train.py --algorithm PPO --timesteps 1000000

# 使用 SAC 训练
python examples/tasks/stair_climbing/train.py --algorithm SAC --timesteps 500000
```

### 3. 评估模型

```bash
python examples/tasks/stair_climbing/evaluate.py --model models/stair_climbing_ppo.zip
```

---

## 性能 Baseline

| 任务 | 算法 | 成功�?| 平均奖励 | 训练时间 |
|------|------|--------|---------|---------|
| 楼梯攀�?| PPO | 85% | 12.5 | 2h (A100) |
| 楼梯攀�?| SAC | 78% | 11.2 | 3h (A100) |

---

## 贡献新任�?

欢迎贡献新任�? 请遵循以下结�?

```
examples/tasks/your_task/
├── env.py          # Gymnasium 环境
├── train.py        # 训练脚本
├── evaluate.py     # 评估脚本
├── README.md       # 任务说明
└── configs/        # 配置文件
    └── default.yaml
```

---

## 引用

如果您在研究中使用了这些任务,请引�?

```bibtex
@software{agi_walker_tasks,
  title = {AGI-Walker Standard Task Suite},
  author = {AGI-Walker Team},
  year = {2026},
  url = {https://github.com/sossossal/AGI-Walker}
}
```

---

**更新日期**: 2026-01-21  
**版本**: v1.0
