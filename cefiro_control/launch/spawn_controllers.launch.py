#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    # Controllers configuration file
    controllers_file = PathJoinSubstitution([
        FindPackageShare('cefiro_control'),
        'config',
        'controller_manager.yaml'
    ])

    # Joint state broadcaster:
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )

    # Motor voltage controller:
    cefiro_attitude_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'cefiro_attitude_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--param-file',
            controllers_file
        ],
        output='screen'
    )


    return LaunchDescription([
        joint_state_broadcaster,
        cefiro_attitude_controller
    ])