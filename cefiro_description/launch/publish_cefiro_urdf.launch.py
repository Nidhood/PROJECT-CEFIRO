#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('cefiro_description')
    model = LaunchConfiguration('model')
    config_file = LaunchConfiguration('physical_config_file')
    visual_mode = LaunchConfiguration('visual_mode')
    enable_collision = LaunchConfiguration('enable_collision')
    enable_sensors = LaunchConfiguration('enable_sensors')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' physical_config_file:=', config_file,
            ' visual_mode:=', visual_mode,
            ' enable_collision:=', enable_collision,
            ' enable_sensors:=', enable_sensors,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([pkg, 'urdf', 'cefiro-drone.urdf.xacro']),
            description='Absolute path to the root Xacro',
        ),
        DeclareLaunchArgument(
            'physical_config_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'cefiro_params.yaml']),
            description='Physical and visual YAML configuration',
        ),
        DeclareLaunchArgument(
            'visual_mode', default_value='separated', choices=['preview', 'kinematic', 'separated'],
            description='Show the actual STL meshes (the other legacy values have no effect)',
        ),
        DeclareLaunchArgument(
            'enable_collision', default_value='true', choices=['true', 'false'],
            description='Use simplified box collision geometry',
        ),
        DeclareLaunchArgument(
            'enable_sensors', default_value='true', choices=['true', 'false'],
            description='Mount onboard FMU IMU, magnetometer and barometer',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use /clock from a simulator',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='cefiro_robot_state_publisher',
            output='screen', parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
