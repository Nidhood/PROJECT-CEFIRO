#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('cefiro_actuators'), 'config', 'actuators.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='cefiro_actuators',
            executable='cefiro_actuators_node',
            name='cefiro_actuators',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            # ROS private topics expand to /cefiro_actuators/{target_rpm,...}
        ),
    ])
