#!/usr/bin/env python3
"""
AGI-Walker ROS 2 桥接节点

连接AGI-Walker仿真平台与ROS 2生态系统
提供标准的ROS 2接口访问Godot仿真
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState, Imu, BatteryState
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    from agi_walker_msgs.msg import RobotState
    from agi_walker_msgs.srv import LoadRobot
except ImportError:
    print("Warning: agi_walker_msgs not found, some features will be limited")
    RobotState = None
    LoadRobot = None

import sys
import os
from pathlib import Path

# 添加AGI-Walker python_api到路径
agi_walker_path = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(agi_walker_path))

try:
    from python_api.godot_client import GodotSimulationClient
except ImportError:
    print("Error: Cannot import godot_client. Please check AGI-Walker installation.")
    GodotSimulationClient = None

import threading
import time
from typing import Dict, Optional


class AGIWalkerROS2Bridge(Node):
    """AGI-Walker ROS 2 桥接节点"""
    
    def __init__(self):
        super().__init__('agi_walker_bridge')
        
        # 声明参数
        self.declare_parameters()
        
        # Godot客户端
        self.godot_client: Optional[GodotSimulationClient] = None
        self.latest_data: Dict = {}
        self.simulation_running = False
        
        # 连接到Godot
        self.connect_to_godot()
        
        # QoS配置
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 设置发布器
        self.setup_publishers()
        
        # 设置订阅器
        self.setup_subscribers()
        
        # 设置服务
        self.setup_services()
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 设置定时器
        self.setup_timers()
        
        self.get_logger().info('✅ AGI-Walker ROS 2 Bridge initialized')
        
    def declare_parameters(self):
        """声明所有ROS参数"""
        # 连接参数
        self.declare_parameter('godot_host', '127.0.0.1')
        self.declare_parameter('godot_port', 9999)
        self.declare_parameter('reconnect_timeout', 5.0)
        
        # 发布频率
        self.declare_parameter('joint_state_rate', 50.0)
        self.declare_parameter('robot_state_rate', 20.0)
        self.declare_parameter('tf_rate', 50.0)
        
        # 控制参数
        self.declare_parameter('motor_power_multiplier', 1.0)
        self.declare_parameter('joint_stiffness', 1.0)
        self.declare_parameter('joint_damping', 0.5)
        self.declare_parameter('balance_gain', 1.0)
        
        # PID参数
        self.declare_parameter('pid_kp', 1.0)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)
        
    def connect_to_godot(self):
        """连接到Godot仿真服务器"""
        if GodotSimulationClient is None:
            self.get_logger().error('❌ GodotSimulationClient not available')
            return
            
        host = self.get_parameter('godot_host').value
        port = self.get_parameter('godot_port').value
        
        self.godot_client = GodotSimulationClient(host, port)
        self.godot_client.set_data_callback(self.on_godot_data)
        
        # 在后台线程尝试连接
        def try_connect():
            if self.godot_client.connect(timeout=3.0):
                self.get_logger().info(f'✅ Connected to Godot at {host}:{port}')
            else:
                self.get_logger().warn(f'⚠️  Failed to connect to Godot at {host}:{port}')
                self.get_logger().info('Bridge is running. Will retry connection when services are called.')
                
        connect_thread = threading.Thread(target=try_connect, daemon=True)
        connect_thread.start()
            
    def setup_publishers(self):
        """设置所有发布器"""
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', self.qos_profile
        )
        
        if RobotState is not None:
            self.robot_state_pub = self.create_publisher(
                RobotState, '/robot_state', self.qos_profile
            )
        else:
            self.robot_state_pub = None
            self.get_logger().warn('RobotState publisher not available (custom msgs not built)')
        
        self.battery_pub = self.create_publisher(
            BatteryState, '/battery', 10
        )
        
        self.imu_pub = self.create_publisher(
            Imu, '/imu', self.qos_profile
        )
        
        self.get_logger().info('Publishers initialized')
        
    def setup_subscribers(self):
        """设置所有订阅器"""
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_vel_callback, 10
        )
        
        self.get_logger().info('Subscribers initialized')
        
    def setup_services(self):
        """设置所有服务"""
        self.start_sim_srv = self.create_service(
            Trigger, '/start_simulation',
            self.start_simulation_callback
        )
        
        self.stop_sim_srv = self.create_service(
            Trigger, '/stop_simulation',
            self.stop_simulation_callback
        )
        
        if LoadRobot is not None:
            self.load_robot_srv = self.create_service(
                LoadRobot, '/load_robot',
                self.load_robot_callback
            )
        else:
            self.load_robot_srv = None
            self.get_logger().warn('LoadRobot service not available (custom msgs not built)')
            
        self.get_logger().info('Services initialized')
        
    def setup_timers(self):
        """设置定时发布器"""
        joint_rate = self.get_parameter('joint_state_rate').value
        if joint_rate > 0:
            self.joint_timer = self.create_timer(
                1.0 / joint_rate, self.publish_joint_states
            )
        
        state_rate = self.get_parameter('robot_state_rate').value
        if state_rate > 0 and self.robot_state_pub is not None:
            self.state_timer = self.create_timer(
                1.0 / state_rate, self.publish_robot_state
            )
            
        self.get_logger().info('Timers initialized')
        
    def on_godot_data(self, data: Dict):
        """接收Godot数据回调"""
        self.latest_data = data
        
    def publish_joint_states(self):
        """发布关节状态"""
        if not self.latest_data:
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 从Godot数据提取关节信息
        # 这里是示例，实际需要根据Godot返回的数据格式调整
        msg.name = ['hip_left', 'knee_left', 'hip_right', 'knee_right']
        msg.position = self.latest_data.get('joint_positions', [0.0] * 4)
        msg.velocity = self.latest_data.get('joint_velocities', [0.0] * 4)
        msg.effort = self.latest_data.get('joint_efforts', [0.0] * 4)
        
        self.joint_state_pub.publish(msg)
        
    def publish_robot_state(self):
        """发布机器人整体状态"""
        if not self.latest_data or self.robot_state_pub is None:
            return
            
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # 提取位置和速度
        pos = self.latest_data.get('position', 0.0)
        vel = self.latest_data.get('velocity', 0.0)
        battery = self.latest_data.get('battery', 100.0)
        
        msg.battery_level = float(battery)
        msg.cpu_usage = 0.0  # TODO: 从Godot获取
        msg.temperature = 25.0  # TODO: 从Godot获取
        msg.status = 'RUNNING' if self.simulation_running else 'IDLE'
        
        self.robot_state_pub.publish(msg)
        
    def cmd_vel_callback(self, msg: Twist):
        """速度命令回调"""
        if self.godot_client and self.godot_client.is_connected():
            # 转换Twist消息为Godot参数
            params = {
                'cmd_linear_x': msg.linear.x,
                'cmd_linear_y': msg.linear.y,
                'cmd_angular_z': msg.angular.z
            }
            self.godot_client.update_parameters(params)
            self.get_logger().info(f'Sent cmd_vel: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}')
        else:
            self.get_logger().warn('Cannot send cmd_vel: not connected to Godot')
            
    def start_simulation_callback(self, request, response):
        """启动仿真服务"""
        if self.godot_client and self.godot_client.is_connected():
            robot_config = {}  # 使用默认配置
            success = self.godot_client.start_simulation(robot_config)
            response.success = success
            response.message = '✅ Simulation started' if success else '❌ Failed to start simulation'
            self.simulation_running = success
        else:
            response.success = False
            response.message = '❌ Not connected to Godot'
            
        self.get_logger().info(response.message)
        return response
        
    def stop_simulation_callback(self, request, response):
        """停止仿真服务"""
        if self.godot_client:
            self.godot_client.stop_simulation()
            response.success = True
            response.message = '✅ Simulation stopped'
            self.simulation_running = False
        else:
            response.success = False
            response.message = '❌ No Godot client available'
            
        self.get_logger().info(response.message)
        return response
        
    def load_robot_callback(self, request, response):
        """加载机器人服务"""
        # 转换ROS消息为Godot格式
        parts = [
            {
                'id': p.part_id,
                'type': p.part_type,
                'model': p.model,
                'position': [p.position.x, p.position.y, p.position.z],
                'parameters': list(p.parameters)
            }
            for p in request.parts
        ]
        
        connections = [
            {
                'from': c.from_part,
                'to': c.to_part,
                'type': c.connection_type
            }
            for c in request.connections
        ]
        
        if self.godot_client and self.godot_client.is_connected():
            success = self.godot_client.load_robot_config(parts, connections)
            response.success = success
            response.message = f'✅ Robot "{request.robot_name}" loaded' if success else '❌ Failed to load robot'
        else:
            response.success = False
            response.message = '❌ Not connected to Godot'
            
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AGIWalkerROS2Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
