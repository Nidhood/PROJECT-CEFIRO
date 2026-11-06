#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Path to the YAML file that contains the bridge configuration:
    ros_bridge_config_file = PathJoinSubstitution([
        FindPackageShare('cefiro_gz'),
        'config',
        'bridge_config.yaml'
    ])
    
    # Bridge node using a YAML configuration file to map multiple topics:
    bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': ros_bridge_config_file
            }],
        )
    
    return LaunchDescription([
        bridge_node
    ])