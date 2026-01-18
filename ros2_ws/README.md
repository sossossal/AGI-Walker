# AGI-Walker ROS 2 Integration

AGI-Walker与ROS 2生态系统的集成包。

## 📦 Packages

### agi_walker_msgs
自定义ROS 2消息和服务定义：
- `Part.msg` - 机器人零件定义
- `Connection.msg` - 零件连接关系
- `RobotState.msg` - 机器人整体状态
- `LoadRobot.srv` - 加载机器人配置服务

### agi_walker_ros2
ROS 2桥接节点，连接AGI-Walker仿真与ROS 2：
- 发布关节状态、机器人状态等
- 接收速度命令、关节命令
- 提供启动/停止仿真服务
- 参数服务器集成

## 🚀 快速开始

### 安装依赖

```bash
# 安装ROS 2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 安装AGI-Walker核心依赖
cd path/to/AGI-Walker
pip install -r requirements.txt
```

### 编译Package

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### 运行桥接节点

```bash
# 终端1: 启动Godot仿真（可选，如未启动则桥接节点会等待连接）
cd ../godot_project
godot --headless

# 终端2: 启动ROS 2桥接
ros2 run agi_walker_ros2 bridge_node
```

### 测试

```bash
# 启动仿真
ros2 service call /start_simulation std_srvs/srv/Trigger

# 查看关节状态
ros2 topic echo /joint_states

# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# 停止仿真
ros2 service call /stop_simulation std_srvs/srv/Trigger
```

## 📚 文档

详细文档请参考：
- [ROS 2集成设计文档](../../docs/ROS2_INTEGRATION_DESIGN.md)

## ⚙️ 参数

桥接节点支持以下参数：

```yaml
/agi_walker_bridge:
  ros__parameters:
    godot_host: "127.0.0.1"
    godot_port: 9999
    joint_state_rate: 50.0
    robot_state_rate: 20.0
    motor_power_multiplier: 1.0
    joint_stiffness: 1.0
    joint_damping: 0.5
```

## 🔧 Topics

### 发布 (Published)
- `/joint_states` (sensor_msgs/JointState) - 50Hz
- `/robot_state` (agi_walker_msgs/RobotState) - 20Hz
- `/battery` (sensor_msgs/BatteryState) - 1Hz
- `/imu` (sensor_msgs/Imu) - 100Hz
- `/tf` (tf2_msgs/TFMessage) - 50Hz

### 订阅 (Subscribed)
- `/cmd_vel` (geometry_msgs/Twist)
- `/joint_cmd` (trajectory_msgs/JointTrajectory)

## 🛠️ Services
- `/start_simulation` (std_srvs/Trigger)
- `/stop_simulation` (std_srvs/Trigger)
- `/load_robot` (agi_walker_msgs/LoadRobot)

## 📝 许可证

MIT License - 详见主项目LICENSE文件

## 🤝 贡献

欢迎提交Issue和Pull Request！
