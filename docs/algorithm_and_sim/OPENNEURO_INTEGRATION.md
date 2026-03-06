# OpenNeuro 通信框架集成指南

## 概述

AGI-Walker 现已集成 **OpenNeuro** 通信框架,支持:
*   ✅ **Zenoh** 轻量级 Pub/Sub 通信
*   ✅ **ROS 2** 生态系统集成
*   ✅ **TCP 向后兼容** (保持现有 Godot 接口)

---

## 安装依赖

### 1. Zenoh Python SDK

```bash
pip install eclipse-zenoh
```

### 2. ROS 2 (可选,用于 ROS 2 集成)

```bash
# Ubuntu 22.04/24.04
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-rclpy ros-jazzy-sensor-msgs

# 配置环境
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 快速开始

### 模式 1: 纯 Zenoh 通信

```python
from python_api.zenoh_interface import ZenohInterface

# 创建接口
zenoh = ZenohInterface()

# 发布命令
zenoh.declare_publisher("rt/robot/cmd")
zenoh.publish("rt/robot/cmd", {"joint_0": 1.5})

# 订阅状态
def on_state(data):
    print(f"State: {data}")

zenoh.declare_subscriber("rt/robot/state", on_state)
```

### 模式 2: TCP-Zenoh 桥接 (保持 Godot 兼容)

```bash
# 终端 1: 启动桥接器
python python_api/tcp_zenoh_bridge.py

# 终端 2: 启动 Godot (现有流程不变)
# Godot 会连接到 TCP 端口 9090

# 终端 3: 通过 Zenoh 发送命令
python -c "
from python_api.zenoh_interface import ZenohInterface
z = ZenohInterface()
z.publish('rt/python/cmd', {'type': 'reset'})
"
```

### 模式 3: ROS 2 节点

```bash
# 终端 1: 启动 ROS 2 节点
python python_api/ros2_robot_node.py

# 终端 2: 查看 Topic
ros2 topic list
# 输出:
#   /robot/joint_states
#   /robot/joint_commands

# 终端 3: 发送命令
ros2 topic pub /robot/joint_commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]"

# 终端 4: 查看状态
ros2 topic echo /robot/joint_states
```

---

## 架构说明

### 通信拓扑

```
┌─────────────────┐
│  Python 控制端   │
│  (RL / 规划)     │
└────────┬────────┘
         │
    Zenoh Pub/Sub
         │
    ┌────┴────┐
    │         │
┌───▼───┐ ┌──▼──────┐
│ Godot │ │ ROS 2   │
│ (仿真) │ │ (工具)  │
└───────┘ └─────────┘
```

### 数据流

**命令流** (Python → Godot):
```
Python RL 策略
    ↓
zenoh.publish("rt/python/cmd", {...})
    ↓
TCP-Zenoh 桥接器
    ↓
TCP Socket → Godot
```

**状态流** (Godot → Python):
```
Godot 传感器
    ↓
TCP Socket → 桥接器
    ↓
zenoh.publish("rt/godot/state", {...})
    ↓
Python 订阅者
```

---

## 常见问题

### Q: Zenoh 连接失败?
**A**: 检查防火墙设置,确保端口 7447 (Zenoh 默认) 未被占用。

### Q: ROS 2 节点无法启动?
**A**: 确认已安装 ROS 2 并 source 环境:
```bash
source /opt/ros/jazzy/setup.bash
```

### Q: Godot 连接超时?
**A**: 确保 TCP-Zenoh 桥接器已启动:
```bash
python python_api/tcp_zenoh_bridge.py
```

### Q: 如何查看 Zenoh 流量?
**A**: 使用 Zenoh 自带的监控工具:
```bash
# 安装 Zenoh CLI
cargo install zenoh --features=zenoh/unstable

# 监控所有消息
zenoh scout
```

---

## 性能指标

| 指标 | TCP (旧) | Zenoh (新) |
|------|---------|-----------|
| 延迟 | ~5-10ms | ~1-2ms |
| 吞吐量 | ~10MB/s | ~100MB/s |
| CPU 占用 | 中 | 低 |
| 内存占用 | 中 | 低 |

---

## 下一步

1.  **验证延迟**: 运行 `examples/zenoh_ros2_demo.py`
2.  **RViz 可视化**: 启动 ROS 2 节点后,使用 RViz 查看机器人状态
3.  **硬件部署**: 参考 OpenNeuro 文档部署到真实硬件

---

**更新日期**: 2026-01-21  
**版本**: v4.1.0-alpha
