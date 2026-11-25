#!/usr/bin/env python3
"""Managed Wi-Fi + direct TCP/CF1; same ROS graph and command safeguards as serial."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('cefiro_hardware_interface'), 'config', 'hardware_wifi.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='real'),
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('wifi_auto_connect', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('wifi_ssid', default_value='CEFIRO_FMU_V3'),
        DeclareLaunchArgument('wifi_interface', default_value='auto'),
        DeclareLaunchArgument('wifi_disconnect_on_exit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('wifi_association_retry_s', default_value='2.0'),
        DeclareLaunchArgument('wifi_mode', default_value='auto', choices=['auto', 'network', 'direct']),
        DeclareLaunchArgument('wifi_host', default_value='cefiro.local'),
        DeclareLaunchArgument('wifi_direct_host', default_value='192.168.4.1'),
        DeclareLaunchArgument('wifi_discovery_grace_s', default_value='15.0'),
        DeclareLaunchArgument('wifi_port', default_value='9000'),
        DeclareLaunchArgument('reconnect_s', default_value='2.0'),
        DeclareLaunchArgument('wifi_connect_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('wifi_rx_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('enable_command_tx', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rpm_to_dshot_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_direct_dshot', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('dshot_max', default_value='300'),
        Node(
            package='cefiro_hardware_interface', executable='hardware_node',
            name='cefiro_hardware_interface', namespace=LaunchConfiguration('namespace'),
            output='screen', parameters=[LaunchConfiguration('config'), {
                'transport': 'wifi', 'protocol_mode': 'cf1', 'use_sim_time': False,
                'wifi_auto_connect': ParameterValue(LaunchConfiguration('wifi_auto_connect'), value_type=bool),
                'wifi_ssid': ParameterValue(LaunchConfiguration('wifi_ssid'), value_type=str),
                'wifi_interface': ParameterValue(LaunchConfiguration('wifi_interface'), value_type=str),
                'wifi_disconnect_on_exit': ParameterValue(LaunchConfiguration('wifi_disconnect_on_exit'), value_type=bool),
                'wifi_association_retry_s': ParameterValue(LaunchConfiguration('wifi_association_retry_s'), value_type=float),
                'wifi_mode': LaunchConfiguration('wifi_mode'),
                'wifi_host': ParameterValue(LaunchConfiguration('wifi_host'), value_type=str),
                'wifi_direct_host': ParameterValue(LaunchConfiguration('wifi_direct_host'), value_type=str),
                'wifi_discovery_grace_s': ParameterValue(LaunchConfiguration('wifi_discovery_grace_s'), value_type=float),
                'wifi_port': ParameterValue(LaunchConfiguration('wifi_port'), value_type=int),
                'serial_reconnect_s': ParameterValue(LaunchConfiguration('reconnect_s'), value_type=float),
                'wifi_connect_timeout_s': ParameterValue(LaunchConfiguration('wifi_connect_timeout_s'), value_type=float),
                'wifi_rx_timeout_s': ParameterValue(LaunchConfiguration('wifi_rx_timeout_s'), value_type=float),
                'enable_command_tx': ParameterValue(LaunchConfiguration('enable_command_tx'), value_type=bool),
                'rpm_to_dshot_enabled': ParameterValue(LaunchConfiguration('rpm_to_dshot_enabled'), value_type=bool),
                'allow_direct_dshot': ParameterValue(LaunchConfiguration('allow_direct_dshot'), value_type=bool),
                'dshot_max': ParameterValue(LaunchConfiguration('dshot_max'), value_type=int),
            }],
        ),
    ])
