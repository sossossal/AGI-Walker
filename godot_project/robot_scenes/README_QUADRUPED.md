# 四足机器人 Godot 场景使用指南

## 场景结构

四足机器人场景 (`quadruped_robot.tscn`) 包含:
- 1个主体 (RigidBody3D, 12kg)
- 4条腿，每条3个关节
- 12个 HingeJoint3D (铰链关节)
- IMU 传感器节点

## 关节配置

每条腿的3个关节:
1. **Hip (髋关节)** - Y轴旋转, ±45°
2. **Thigh (大腿关节)** - X轴旋转, -120° to 60°
3. **Shin (小腿关节)** - X轴旋转, 0° to 150°

## 使用方法

### 1. 在 Godot 中加载

```gdscript
# 在你的场景中实例化
var quadruped_scene = load("res://robot_scenes/quadruped_robot.tscn")
var robot = quadruped_scene.instantiate()
add_child(robot)
```

### 2. 控制关节

```gdscript
# 设置12个关节目标
var joint_targets = [
    0.0, 0.0, 0.0,  # FL: hip, thigh, shin
    0.0, 0.0, 0.0,  # FR
    0.0, 0.0, 0.0,  # RL
    0.0, 0.0, 0.0   # RR
]
robot.set_joint_targets(joint_targets)
```

### 3. 获取状态

```gdscript
var state = robot.get_state()
print("Position: ", state["position"])
print("Velocity: ", state["linear_velocity"])
print("Joint angles: ", state["joint_angles"])
```

## Python 集成

通过 TCP 与 Python 通信:

```python
import gymnasium as gym

env = gym.make('AGI-Walker/Quadruped-v0')
obs, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, done, truncated, info = env.step(action)
    if done:
        break
```

## 物理参数

- **总质量**: 12 kg
- **身体**: 0.4m × 0.15m × 0.2m
- **腿长**: 0.3m (大腿0.15m + 小腿0.15m)
- **电机扭矩**: 5.0 Nm (每个关节)

## 调试技巧

1. 在 Godot 中启用 Debug -> Visible Collision Shapes
2. 使用 Remote 标签查看节点属性
3. 检查 Output 面板的错误信息

## 下一步

- 添加足端触觉传感器
- 优化碰撞形状
- 添加更真实的材质
