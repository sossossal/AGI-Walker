"""
ROS 2 机器人节点
将 AGI-Walker 集成到 ROS 2 生态系统
"""

import sys
import json
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, Header
    from builtin_interfaces.msg import Time

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print(
        "⚠️ ROS 2 未安装，请运行: sudo apt install ros-jazzy-rclpy ros-jazzy-sensor-msgs"
    )

# 尝试导入 Zenoh 接口
try:
    from python_api.comm.zenoh_interface import ZenohInterface

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False


class AGIWalkerNode(Node):
    """
    AGI-Walker ROS 2 节点

    功能:
        1. 发布 /robot/joint_states (JointState)
        2. 订阅 /robot/joint_commands (Float64MultiArray)
        3. 通过 Zenoh 与 Godot 通信

    用法:
        ros2 run agi_walker robot_node
        # 或
        python python_api/ros2_robot_node.py
    """

    def __init__(self):
        super().__init__("agi_walker_node")

        # 机器人配置
        self.joint_names = [
            "joint_0",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]

        # ROS 2 发布者
        self.state_pub = self.create_publisher(JointState, "/robot/joint_states", 10)

        # ROS 2 订阅者
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "/robot/joint_commands", self.on_joint_command, 10
        )

        # Zenoh 接口 (可选)
        self.zenoh: Optional[ZenohInterface] = None
        if ZENOH_AVAILABLE:
            try:
                self.zenoh = ZenohInterface()
                self.zenoh.declare_subscriber("rt/godot/state", self.on_godot_state)
                self.zenoh.declare_publisher("rt/python/cmd")
                self.get_logger().info("✅ Zenoh 接口已启用")
            except Exception as e:
                self.get_logger().warn(f"Zenoh 初始化失败: {e}")

        # 定时器 (10Hz 发布频率)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # 当前状态
        self.current_positions = [0.0] * len(self.joint_names)
        self.current_velocities = [0.0] * len(self.joint_names)
        self.current_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info("🤖 AGI-Walker ROS 2 节点已启动")

    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        msg.effort = self.current_efforts

        self.state_pub.publish(msg)

    def on_joint_command(self, msg: Float64MultiArray):
        """接收关节命令"""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                f"命令长度不匹配: 期望 {len(self.joint_names)}, 收到 {len(msg.data)}"
            )
            return

        # 转发到 Zenoh (Godot)
        if self.zenoh:
            cmd = {"type": "joint_command", "positions": list(msg.data)}
            self.zenoh.publish("rt/python/cmd", cmd)

        self.get_logger().info(f"📤 发送命令: {msg.data[:3]}...")

    def on_godot_state(self, data):
        """接收 Godot 状态 (通过 Zenoh)"""
        try:
            # 更新当前状态
            if "joint_positions" in data:
                self.current_positions = data["joint_positions"]
            if "joint_velocities" in data:
                self.current_velocities = data["joint_velocities"]
            if "joint_efforts" in data:
                self.current_efforts = data["joint_efforts"]
        except Exception as e:
            self.get_logger().error(f"解析 Godot 状态失败: {e}")

    def destroy_node(self):
        """清理资源"""
        if self.zenoh:
            self.zenoh.close()
        super().destroy_node()


def main(args=None):
    if not ROS2_AVAILABLE:
        print("❌ ROS 2 未安装，无法运行")
        return

    rclpy.init(args=args)
    node = AGIWalkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
