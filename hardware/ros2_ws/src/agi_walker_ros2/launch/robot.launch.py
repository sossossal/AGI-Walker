#!/usr/bin/env python3
"""
AGI-Walker ROS 2 launch wrapper.

Starts the bridge launch and optionally adds robot_state_publisher and RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("agi_walker_ros2")
    bridge_launch_file = os.path.join(pkg_share, "launch", "agi_walker.launch.py")
    urdf_file = os.path.join(pkg_share, "urdf", "agi_walker.urdf")

    with open(urdf_file, encoding="utf-8") as handle:
        robot_description = handle.read()

    godot_host = LaunchConfiguration("godot_host")
    godot_port = LaunchConfiguration("godot_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    start_rviz = LaunchConfiguration("start_rviz")

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_file),
        launch_arguments={
            "godot_host": godot_host,
            "godot_port": godot_port,
            "use_sim_time": use_sim_time,
            "config_file": config_file,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(publish_robot_description),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "godot_host",
                default_value="127.0.0.1",
                description="Godot simulation server host",
            ),
            DeclareLaunchArgument(
                "godot_port",
                default_value="9999",
                description="Godot simulation server port",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(pkg_share, "config", "params.yaml"),
                description="Bridge parameter YAML file",
            ),
            DeclareLaunchArgument(
                "publish_robot_description",
                default_value="true",
                description="Start robot_state_publisher with the packaged URDF",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Start RViz without a custom config",
            ),
            bridge_launch,
            robot_state_publisher,
            rviz_node,
        ]
    )
