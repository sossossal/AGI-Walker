# 双足行走机器人完整案例

这是一个完整的端到端案例，展示如何使用 AGI-Walker 开发双足机器人，从零件选型到硬件部署。

---

## 📋 项目概览

**目标**: 训练一个能够稳定行走的双足机器人

**特性**:
- ✅ 12 个关节（每条腿 6 个）
- ✅ 基于真实零件规格
- ✅ 完整的 Sim-to-Real 工作流
- ✅ 成本透明

---

## 🤖 机器人设计

### 硬件配置

| 部位 | 零件 | 数量 | 单价 | 小计 |
|------|------|------|------|------|
| 髋关节 (俯仰) | Dynamixel MX-106 | 2 | $489.9 | $979.8 |
| 髋关节 (偏航/滚转) | Dynamixel XL430 | 4 | $49.9 | $199.6 |
| 膝关节 | Dynamixel MX-106 | 2 | $489.9 | $979.8 |
| 踝关节 | Dynamixel XL430 | 4 | $49.9 | $199.6 |
| IMU | MPU6050 | 1 | $3.5 | $3.5 |
| 控制器 | IMC-22 × 12 | 12 | $40 | $480 |
| **总计** | - | - | - | **$2,842.3** |

### 机器人规格

- **高度**: 60 cm
- **重量**: ~5 kg
- **最大速度**: 1.0 m/s（设计目标）
- **电池续航**: 30 分钟（估算）

---

## 📂 项目文件

```
walker_biped/
├── robot_config.json        # 机器人配置
├── train.py                 # 训练脚本
├── deploy.py                # 部署脚本
├── README.md                # 本文档
└── godot_scene/             # Godot 场景文件
    └── biped_walker.tscn
```

---

## 🚀 使用指南

### 步骤 1: 查看配置

```bash
cd examples/walker_biped
python -c "import json; print(json.dumps(json.load(open('robot_config.json')), indent=2, ensure_ascii=False))"
```

### 步骤 2: 训练策略

```bash
# 基础训练（较快，适合测试）
python train.py --mode=basic --timesteps=100000

# 完整训练（需要更长时间）
python train.py --mode=full --timesteps=1000000

# 使用域随机化
python train.py --mode=full --domain-randomization --timesteps=500000
```

**预期训练时间**:
- 基础模式: ~30 分钟
- 完整模式: ~4 小时
- 域随机化: ~2 小时

### 步骤 3: 测试策略

```bash
python train.py --test --model=walker_policy.zip
```

### 步骤 4: 部署到硬件

```bash
python deploy.py --model=walker_policy.zip --quantize
```

---

## 📊 性能指标

训练后预期达到的性能：

| 指标 | 目标 | 实际 |
|------|------|------|
| 前进速度 | 0.8 m/s | TBD |
| 稳定性 | 90% 不跌倒 | TBD |
| 能耗效率 | < 100W | TBD |
| 响应延迟 | < 100ms | TBD |

---

## 🎓 学习要点

### 1. 零件选择策略

**大关节（髋、膝）**:
- 选择高扭矩电机（MX-106: 8.4 N·m）
- 承受主要重量和加速度

**小关节（踝、辅助）**:
- 选择性价比高的电机（XL430: 1.0 N·m）
- 精细调整和平衡

### 2. 训练技巧

**奖励函数设计**:
```python
reward = (
    forward_velocity * 1.0      # 鼓励前进
    - abs(lateral_velocity) * 0.5  # 惩罚侧向移动
    - abs(angular_velocity) * 0.5  # 惩罚旋转
    - energy_consumption * 0.01    # 惩罚能耗
    + upright_bonus * 0.2          # 鼓励站立
)
```

**课程学习**:
1. 阶段 1: 学习站立（100K steps）
2. 阶段 2: 学习迈步（200K steps）
3. 阶段 3: 学习行走（500K steps）
4. 阶段 4: 优化步态（200K steps）

### 3. Sim-to-Real 技巧

**域随机化参数**:
- 地面摩擦: 0.5 - 1.5
- 关节摩擦: 0.8 - 1.2
- 传感器噪声: ±5%
- 通信延迟: 0 - 10ms
- 重力: 9.5 - 10.2 m/s²

---

## 🔧 故障排查

### 问题 1: 机器人无法站立

**可能原因**:
- 重心过高
- PID 参数不当
- 传感器校准问题

**解决方案**:
1. 降低重心（调整质量分布）
2. 调整 PID 参数
3. 重新校准 IMU

### 问题 2: 行走不稳定

**可能原因**:
- 步态周期不合理
- 地面接触检测失败
- 平衡控制不足

**解决方案**:
1. 增加训练时间
2. 调整奖励函数
3. 使用域随机化

### 问题 3: 硬件性能不佳

**可能原因**:
- Sim-to-Real gap
- 模型量化损失
- 通信延迟

**解决方案**:
1. 在真实硬件上微调
2. 使用 QAT（量化感知训练）
3. 优化 CAN 总线配置

---

## 📹 演示视频

（录制后添加链接）

- 训练过程时间线
- 行走步态演示
- 不同地形测试
- 硬件部署实况

---

## 🔗 相关资源

- [完整训练日志](training_logs/)
- [性能对比图表](performance_charts/)
- [硬件组装指南](assembly_guide.md)
- [3D 打印文件](3d_models/)

---

## 📞 获取帮助

遇到问题？
- 💬 Discord: #biped-walker 频道
- 📧 邮箱: support@agi-walker.org
- 🐛 GitHub Issues

---

**案例版本**: 1.0  
**创建日期**: 2026-01-16  
**维护者**: AGI-Walker Team
