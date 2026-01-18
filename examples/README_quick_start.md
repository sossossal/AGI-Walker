# 快速开始示例：平衡机器人

这个示例演示如何使用 AGI-Walker 从零开始开发一个简单的平衡机器人。

## 运行示例

```bash
cd examples
python quick_start_balance.py
```

## 示例包含

1. ✅ **零件库查询** - 浏览可用的电机和传感器
2. ✅ **机器人设计** - 配置 2 关节平衡机器人
3. ✅ **成本估算** - 计算硬件总成本
4. ✅ **仿真训练** - 使用 PPO 训练平衡策略
5. ✅ **性能测试** - 在仿真中验证效果
6. ✅ **硬件部署** - (可选) 导出到真实硬件

## 预期输出

```
==============================================================
AGI-Walker 快速开始示例：平衡机器人
==============================================================

步骤 1: 浏览零件库
--------------------------------------------------------------

可用电机:
  - Dynamixel XL430-W250-T
    扭矩: 1.0 N·m
    价格: $49.9
  - Dynamixel MX-106T
    扭矩: 8.4 N·m
    价格: $489.9

步骤 2: 设计简单平衡机器人
--------------------------------------------------------------
机器人配置: SimpleBalancer
关节数量: 2
总成本: $99.80

步骤 3: 创建仿真环境
--------------------------------------------------------------
观察空间: Box(-inf, inf, (6,), float32)
动作空间: Box(-1.0, 1.0, (2,), float32)

步骤 4: 训练平衡策略
--------------------------------------------------------------
开始训练... (这可能需要几分钟)
| rollout/           |          |
|    ep_len_mean     | 100      |
|    ep_rew_mean     | 250.5    |
...

✓ 模型已保存: balance_policy.zip

步骤 5: 测试训练好的策略
--------------------------------------------------------------
回合结束 - 总奖励: 987.35

步骤 6: (可选) 导出到硬件
--------------------------------------------------------------
运行以下命令部署到 IMC-22 硬件:
  python examples/deploy_to_hardware.py

==============================================================
完成！您已经:
  ✓ 选择了硬件零件
  ✓ 设计了机器人
  ✓ 训练了控制策略
  ✓ 测试了性能

下一步:
  - 尝试不同的环境设置（月球重力、冰面等）
  - 使用域随机化提高鲁棒性
  - 部署到真实硬件
==============================================================
```

## 学习要点

### 1. 零件库使用

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase()
motor = db.get_part("dynamixel_xl430_w250")
print(f"扭矩: {motor['specifications']['stall_torque']} N·m")
```

### 2. 环境创建

```python
env = GodotRobotEnv(
    scene_name="balance_bot",
    env_preset="earth",
    ground_material="concrete"
)
```

### 3. PPO 训练

```python
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)
model.save("balance_policy")
```

### 4. 策略测试

```python
obs, info = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
```

## 下一步

- 📚 查看 [进阶使用指南](../ADVANCED_USAGE.md)
- 🎮 尝试 [环境系统](../PHYSICS_ENVIRONMENT_GUIDE.md)
- 🤖 部署到 [真实硬件](../HARDWARE_INTEGRATION_GUIDE.md)
