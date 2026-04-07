#!/usr/bin/env python3
"""
AGI-Walker ROS 2 Launch文件

启动AGI-Walker ROS 2桥接节点和相关组件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package目录
    pkg_share = get_package_share_directory('agi_walker_ros2')
    
    # Launch参数
    godot_host_arg = DeclareLaunchArgument(
        'godot_host',
        default_value='127.0.0.1',
        description='Godot simulation server host'
    )
    
    godot_port_arg = DeclareLaunchArgument(
        'godot_port',
        default_value='9999',
        description='Godot simulation server port'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    # AGI-Walker桥接节点
    bridge_node = Node(
        package='agi_walker_ros2',
        executable='bridge_node',
        name='agi_walker_bridge',
        output='screen',
        parameters=[
            config_file,
            {
                'godot_host': LaunchConfiguration('godot_host'),
                'godot_port': LaunchConfiguration('godot_port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        emulate_tty=True
    )
    
    # Robot State Publisher (可选 - 用于发布TF)
    # 注意：需要URDF文件，这里先注释掉
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )
    
    return LaunchDescription([
        # 参数声明
        godot_host_arg,
        godot_port_arg,
        use_sim_time_arg,
        
        # 节点
        bridge_node,
        
        # 延迟启动其他节点（可选）
        # TimerAction(
        #     period=2.0,
        #     actions=[robot_state_publisher]
        # )
    ])
