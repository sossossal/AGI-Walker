# 物理参数优化指南

本文档说明如何优化AGI-Walker项目的物理参数，提升仿真真实感和机器人稳定性。

---

## 📋 已实现的优化

### 1. 物理参数配置系统

创建了 `physics_config.gd` 集中管理所有物理参数：

#### 关键参数说明

| 参数类别 | 参数名 | 默认值 | 说明 |
|---------|--------|--------|------|
| **地面** | `GROUND_FRICTION` | 0.8 | 摩擦系数（越高越防滑） |
| | `GROUND_BOUNCE` | 0.0 | 弹性系数（0=不弹） |
| **躯干** | `TORSO_MASS` | 10.0 kg | 质量 |
| | `TORSO_LINEAR_DAMP` | 0.1 | 线性阻尼（空气阻力） |
| | `TORSO_ANGULAR_DAMP` | 0.5 | 角阻尼（旋转阻力） |
| **腿部** | `LEG_MASS` | 3.0 kg | 质量（增加以提高稳定性） |
| | `FOOT_FRICTION` | 0.9 | 脚底摩擦力 |
| **关节** | `HIP_LIMIT_LOWER` | -45° | 髋关节下限 |
| | `HIP_LIMIT_UPPER` | 90° | 髋关节上限 |
| | `MOTOR_MAX_IMPULSE` | 500 N·m | 最大扭矩 |
| | `JOINT_DAMPING` | 0.5 | 关节阻尼（抑制震荡） |

---

## 🚀 使用方法

### 方式1: 自动优化（推荐）

1. 在Godot编辑器中，将 `RobotOptimizer` 节点添加到主场景：
   ```
   Main (Node3D)
   ├── ...
   └── RobotOptimizer (Node) [脚本: robot_optimizer.gd]
   ```

2. 运行场景，自动应用优化参数

3. 查看控制台输出：
   ```
   🔧 开始优化物理参数...
   ✅ 地面物理材质已应用: 摩擦力=0.80
   ✅ 躯干参数已应用: 质量=10.0kg
   ✅ 关节参数已应用: 限位=[-45°, 90°]
   ```

### 方式2: 手动应用

在任何GDScript中调用：

```gdscript
# 应用到地面
PhysicsConfig.apply_to_ground($Ground)

# 应用到机器人组件
PhysicsConfig.apply_to_torso($Robot/Torso)
PhysicsConfig.apply_to_leg($Robot/LeftLeg)
PhysicsConfig.apply_to_hip_joint($Robot/HipLeft)
```

---

## 🧪 测试与调优

### Python测试工具

使用 `physics_tuner.py` 自动化测试参数效果：

```bash
cd python_controller
python physics_tuner.py
```

**功能**:
- ✅ 稳定性测试（测量倾斜角、持续时间）
- ✅ 电机响应速度测试
- ✅ 参数扫描（批量测试多组配置）

**输出示例**:
```
🧪 开始稳定性测试 (持续30秒)...

📊 测试结果:
  持续时间: 30.00s
  平均倾斜: 8.32°
  最大倾斜: 15.47°
  稳定性评分: 91.7/100
```

---

## 🔧 调优建议

### 问题1: 机器人不稳定，容易摔倒

**可能原因**:
- 重心过高
- 腿部质量太轻
- 阻尼不足

**解决方案**:
1. 增加腿部质量: `LEG_MASS = 5.0`
2. 增加角阻尼: `TORSO_ANGULAR_DAMP = 1.0`
3. 降低重心（在Godot中调整Torso位置）

### 问题2: 机器人打滑

**解决方案**:
1. 增加地面摩擦: `GROUND_FRICTION = 1.0`
2. 增加脚底摩擦: `FOOT_FRICTION = 1.0`
3. 降低电机扭矩避免过力: `MOTOR_MAX_IMPULSE = 300`

### 问题3: 关节震荡/抖动

**解决方案**:
1. 增加关节阻尼: `JOINT_DAMPING = 0.8`
2. 启用软限位: `HIP_LIMIT_SOFTNESS = 0.9`
3. 降低电机响应速度: `MOTOR_SPEED_MULTIPLIER = 3.0`

### 问题4: 电机响应太慢

**解决方案**:
1. 增加扭矩: `MOTOR_MAX_IMPULSE = 800`
2. 增加响应速度: `MOTOR_SPEED_MULTIPLIER = 8.0`
3. 减少关节阻尼: `JOINT_DAMPING = 0.2`

---

## 📊 性能基准

### 理想指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 平均倾斜角 | < 10° | 越小越稳定 |
| 稳定性评分 | > 85/100 | 综合评价 |
| 电机响应时间 | < 0.5s | 到达目标角度的时间 |
| 持续站立时间 | > 30s | 无AI控制时的自然站立 |

---

## 🎓 物理参数详解

### 阻尼 (Damping)

**线性阻尼**: 模拟空气阻力，阻止线性运动
- 0.0 = 无阻力（真空）
- 1.0 = 高阻力（类似水中）

**角阻尼**: 模拟旋转阻力
- 用于抑制不必要的旋转
- 提高姿态稳定性

### 摩擦力 (Friction)

- COF (系数) = F_friction / F_normal
- 典型值:
  - 冰面: 0.05
  - 木地板: 0.4
  - 橡胶: 0.8-1.0

### 软限位 (Soft Limits)

传统硬限位会在边界突然停止，导致震荡。软限位使用弹簧-阻尼系统柔和地限制范围。

参数:
- `SOFTNESS`: 0=硬限位, 1=极软
- `BIAS`: 恢复力强度

---

## 🔬 高级调优

### 自定义参数配置

修改 `physics_config.gd` 中的常量：

```gdscript
# 示例: 创建"重型机器人"配置
const TORSO_MASS = 20.0  # 加倍质量
const LEG_MASS = 6.0
const MOTOR_MAX_IMPULSE = 1000.0  # 加强电机
```

### 动态参数调整

在运行时通过Python客户端调整参数（高级功能，需自行实现）：

```python
# 伪代码示例
client.send_config({
    "motor_force": 600,
    "damping": 0.7
})
```

---

## ✅ 验证清单

优化完成后，检查以下项目:

- [ ] 控制台无错误/警告
- [ ] 机器人站立超过30秒不摔倒
- [ ] 平均倾斜角 < 10°
- [ ] 电机能响应Python指令
- [ ] 关节无明显震荡
- [ ] 脚底无滑动（接触传感器正常）

---

## 📚 参考资源

- [Godot Physics文档](https://docs.godotengine.org/en/stable/tutorials/physics/index.html)
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics)
- 机器人学经典教材：《Modern Robotics》

---

> 💡 **提示**: 参数调优是一个迭代过程。建议从默认配置开始，逐步微调，并使用`physics_tuner.py`记录每次测试结果。
