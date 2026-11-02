#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('cefiro_hardware_interface'), 'config', 'hardware.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='real'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('protocol_mode', default_value='legacy_text',
                              choices=['legacy_text', 'cf1']),
        DeclareLaunchArgument('enable_command_tx', default_value='false',
                              choices=['true', 'false']),
        Node(
            package='cefiro_hardware_interface',
            executable='hardware_node',
            name='cefiro_hardware_interface',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[default_config, LaunchConfiguration('config'), {
                'port': LaunchConfiguration('port'),
                'protocol_mode': LaunchConfiguration('protocol_mode'),
                'enable_command_tx': ParameterValue(LaunchConfiguration('enable_command_tx'), value_type=bool),
                'use_sim_time': False,
            }],
        ),
    ])
