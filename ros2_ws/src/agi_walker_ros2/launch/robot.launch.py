"""
AGI-Walker ROS 2 Launch File
启动完整的机器人系统: 节点 + 状态发布 + RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('agi_walker_ros2').find('agi_walker_ros2')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    )
    urdf_file = LaunchConfiguration(
        'urdf_file',
        default=os.path.join(pkg_share, 'urdf', 'agi_walker.urdf')
    )
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='RViz config file path'
    )
    
    # 机器人节点
    robot_node = Node(
        package='agi_walker_ros2',
        executable='robot_node',
        name='agi_walker_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 状态发布器 (发布 TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(urdf_file.perform(None)).read()}
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        robot_node,
        robot_state_publisher,
        rviz_node
    ])
