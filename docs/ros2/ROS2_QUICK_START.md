# AGI-Walker ROS 2 集成快速入门指�?

本指南帮助您快速开始使用AGI-Walker的ROS 2集成功能�?

---

## 📋 前提条件

### 系统要求
- **操作系统**: Ubuntu 22.04 LTS（推荐）
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Godot**: 4.x（可选，用于3D仿真�?

### 安装ROS 2

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS 2 apt�?
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

---

## 🚀 快速开�?

### 步骤1: 克隆仓库

```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
```

### 步骤2: 安装AGI-Walker依赖

```bash
pip install -r requirements.txt
pip install matplotlib pillow  # GUI依赖
```

### 步骤3: 编译ROS 2 Packages

```bash
cd hardware/ros2_ws

# Source ROS 2环境
source /opt/ros/humble/setup.bash

# 编译
colcon build

# Source工作空间
source install/setup.bash
```

### 步骤4: 启动桥接节点

```bash
# 方式1: 使用launch文件（推荐）
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 方式2: 直接运行节点
ros2 run agi_walker_ros2 bridge_node
```

---

## 🧪 测试功能

### 1. 查看Topics

```bash
# 列出所有topics
ros2 topic list

# 查看关节状�?
ros2 topic echo /joint_states

# 查看机器人状�?
ros2 topic echo /robot_state
```

### 2. 调用Services

```bash
# 启动仿真
ros2 service call /start_simulation std_srvs/srv/Trigger

# 停止仿真
ros2 service call /stop_simulation std_srvs/srv/Trigger
```

### 3. 发送命�?

```bash
# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" \
  --once

# 或者持续发�?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### 4. 查看和修改参�?

```bash
# 列出所有参�?
ros2 param list /agi_walker_bridge

# 查看参数�?
ros2 param get /agi_walker_bridge motor_power_multiplier

# 修改参数
ros2 param set /agi_walker_bridge motor_power_multiplier 1.5

# 从文件加载参�?
ros2 param load /agi_walker_bridge src/agi_walker_ros2/config/params.yaml
```

---

## 🎮 完整使用流程

### 场景1: 纯模拟（无Godot�?

```bash
# 终端1: 启动桥接节点
cd AGI-Walker/hardware/ros2_ws
source install/setup.bash
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 终端2: 发送命�?
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

桥接节点会等待Godot连接，但仍可接收ROS命令�?

### 场景2: 与Godot仿真集成

```bash
# 终端1: 启动Godot仿真（在AGI-Walker目录�?
cd AGI-Walker/godot_project
godot --headless  # 或者用GUI启动

# 终端2: 启动ROS 2桥接
cd AGI-Walker/hardware/ros2_ws
source install/setup.bash
ros2 launch agi_walker_ros2 agi_walker.launch.py

# 终端3: 启动仿真
ros2 service call /start_simulation std_srvs/srv/Trigger

# 终端4: 查看实时数据
ros2 topic echo /joint_states
```

### 场景3: 使用Python脚本控制

创建文件 `test_control.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_client = self.create_client(Trigger, '/start_simulation')
        
    def start_sim(self):
        req = Trigger.Request()
        future = self.start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def move(self, linear_x, angular_z, duration=1.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

def main():
    rclpy.init()
    controller = RobotController()
    
    # 启动仿真
    controller.get_logger().info('Starting simulation...')
    result = controller.start_sim()
    controller.get_logger().info(f'Result: {result.message}')
    
    # 前进
    controller.get_logger().info('Moving forward...')
    controller.move(0.5, 0.0, 2.0)
    
    # 转向
    controller.get_logger().info('Turning...')
    controller.move(0.0, 0.5, 2.0)
    
    controller.get_logger().info('Done!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

运行:
```bash
chmod +x test_control.py
python3 test_control.py
```

---

## 🔧 参数配置

编辑 `hardware/hardware/hardware/ros2_ws/src/agi_walker_ros2/config/params.yaml`:

```yaml
/agi_walker_bridge:
  ros__parameters:
    # 修改Godot连接
    godot_host: "192.168.1.100"  # 远程Godot服务�?
    godot_port: 9999
    
    # 调整发布频率
    joint_state_rate: 100.0  # 提高�?00Hz
    
    # 调整控制参数
    motor_power_multiplier: 1.5
    joint_stiffness: 2.0
```

重新启动节点以应用更改�?

---

## 🐛 故障排查

### 问题1: 无法找到package

**错误**: `Package 'agi_walker_ros2' not found`

**解决**:
```bash
cd hardware/ros2_ws
colcon build
source install/setup.bash
```

### 问题2: 连接Godot失败

**错误**: `Failed to connect to Godot`

**检�?*:
1. Godot是否运行�?
2. TCP服务器是否启动（端口9999）？
3. 防火墙是否阻止连接？

**测试连接**:
```bash
telnet 127.0.0.1 9999
```

### 问题3: 自定义消息未找到

**错误**: `ModuleNotFoundError: No module named 'agi_walker_msgs'`

**解决**:
```bash
# 确保编译了消息package
cd hardware/ros2_ws
colcon build --packages-select agi_walker_msgs
source install/setup.bash
```

### 问题4: Python路径问题

**错误**: `Cannot import godot_client`

**解决**:
```bash
# 确保AGI-Walker在Python路径�?
export PYTHONPATH=$PYTHONPATH:/path/to/AGI-Walker
```

---

## 📊 可视�?

### 使用rqt查看数据

```bash
# 安装rqt工具
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# 启动rqt
rqt
```

在rqt�?
- Plugins �?Topics �?Topic Monitor - 查看所有topics
- Plugins �?Visualization �?Plot - 绘制数据曲线
- Plugins �?Services �?Service Caller - 调用服务

### 使用Plotjuggler

```bash
# 安装
sudo apt install ros-humble-plotjuggler-ros

# 启动
ros2 run plotjuggler plotjuggler
```

---

## 🎯 下一�?

1. **尝试RViz可视�?* (需要TF系统，Phase 2)
2. **集成MoveIt运动规划** (未来功能)
3. **创建自定义控制器**
4. **连接真实硬件**

---

## 📚 更多资源

- [ROS 2官方文档](https://docs.ros.org/en/humble/)
- [AGI-Walker项目主页](https://github.com/sossossal/AGI-Walker)
- [ROS 2集成设计文档](../docs/ROS2_INTEGRATION_DESIGN.md)

---

**遇到问题�?* 请在GitHub上提交Issue�?
