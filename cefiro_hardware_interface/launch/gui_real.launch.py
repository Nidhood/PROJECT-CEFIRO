#!/usr/bin/env python3
"""CEFIRO dual-plant GUI with independent SIM and REAL topics and wall time."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('cefiro_gui'), 'config', 'gui_config.yaml'
    ])
    return LaunchDescription([
        Node(package='cefiro_gui', executable='cefiro_gui_node',
             name='cefiro_hmi', output='screen',
             parameters=[config, {'use_sim_time': False}]),
    ])
