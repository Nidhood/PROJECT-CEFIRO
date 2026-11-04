#!/usr/bin/env python3
"""ROS 2 launch description. The hardware node and serial protocol are C++."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('cefiro_hardware_interface'), 'config', 'hardware.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='real'),
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('protocol_mode', default_value='legacy_text',
                              choices=['legacy_text', 'cf1']),
        DeclareLaunchArgument('enable_command_tx', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rpm_to_dshot_enabled', default_value='false',
                              choices=['true', 'false']),
        # Explicit host-side upper throttle code. The STM32 protocol cap is 2047.
        # Leave at 300 until motor, ESC, power and mounting are validated.
        DeclareLaunchArgument('dshot_max', default_value='300'),
        Node(
            package='cefiro_hardware_interface',
            executable='hardware_node',
            name='cefiro_hardware_interface',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'port': LaunchConfiguration('port'),
                'protocol_mode': LaunchConfiguration('protocol_mode'),
                'enable_command_tx': ParameterValue(
                    LaunchConfiguration('enable_command_tx'), value_type=bool),
                'rpm_to_dshot_enabled': ParameterValue(
                    LaunchConfiguration('rpm_to_dshot_enabled'), value_type=bool),
                'dshot_max': ParameterValue(
                    LaunchConfiguration('dshot_max'), value_type=int),
                'use_sim_time': False,
            }],
        ),
    ])
