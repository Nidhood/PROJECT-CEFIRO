#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('cefiro_gui'), 'config', 'gui_config.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        Node(
            package='cefiro_gui',
            executable='cefiro_gui_node',
            name='cefiro_hmi',
            output='screen',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
