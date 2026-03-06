# 模型发布指南

## 预训练模型库

AGI-Walker 提供预训练的强化学习模型,可直接用于评估或微调。

### 可用模型

| 任务 | 算法 | 训练步数 | 成功率 | 下载 |
|------|------|---------|--------|------|
| 楼梯攀爬 | PPO | 1M | 85% | [下载](models/stair_climbing/final_model.zip) |
| 崎岖地形 | PPO | 1M | 78% | 训练中 |
| 物体抓取 | SAC | 500K | 72% | 训练中 |

### 快速开始

#### 1. 下载模型
```bash
# 下载楼梯攀爬模型
wget https://github.com/sossossal/AGI-Walker/releases/download/v4.2.0/stair_climbing_ppo.zip
```

#### 2. 加载模型
```python
from stable_baselines3 import PPO
import gymnasium as gym

# 加载模型
model = PPO.load("stair_climbing_ppo.zip")

# 创建环境
env = gym.make('StairClimbing-v0')

# 运行
obs, info = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break

print(f"Steps climbed: {info['steps_climbed']}/5")
```

#### 3. 微调模型
```python
# 继续训练
model.learn(total_timesteps=100000)
model.save("finetuned_model.zip")
```

### 训练自己的模型

```bash
# 安装依赖
pip install stable-baselines3

# 训练楼梯攀爬
python examples/tasks/stair_climbing/train.py --timesteps 1000000

# 评估模型
python examples/tasks/stair_climbing/train.py --mode eval --model models/stair_climbing/final_model.zip
```

### 性能 Baseline

#### 楼梯攀爬 (Stair Climbing)
- **算法**: PPO
- **训练步数**: 1,000,000
- **成功率**: 85%
- **平均奖励**: 12.5 ± 2.3
- **训练时间**: ~2 小时 (NVIDIA A100)

**奖励曲线**:
```
Episode 0-100:    平均奖励 -5.2
Episode 100-500:  平均奖励 3.8
Episode 500-1000: 平均奖励 12.5
```

### 贡献模型

欢迎贡献预训练模型!

1. 训练模型
2. 评估性能 (>70% 成功率)
3. 提交 Pull Request
4. 包含:
   - 模型文件 (.zip)
   - 训练统计 (training_stats.json)
   - 评估结果

### 模型许可

所有预训练模型采用 MIT 许可证,可自由使用、修改和分发。
