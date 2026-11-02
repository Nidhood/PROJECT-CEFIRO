#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('cefiro_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('use_joint_state_gui')

    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'publish_cefiro_urdf.launch.py'])
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'physical_config_file': LaunchConfiguration('physical_config_file'),
            'use_sim_time': use_sim_time,
            'visual_mode': LaunchConfiguration('visual_mode'),
            'enable_collision': LaunchConfiguration('enable_collision'),
            'enable_sensors': LaunchConfiguration('enable_sensors'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_joint_state_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('visual_mode', default_value='separated', choices=['preview', 'kinematic', 'separated']),
        DeclareLaunchArgument('enable_collision', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_sensors', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([pkg, 'urdf', 'cefiro-gyro.urdf.xacro'])
        ),
        DeclareLaunchArgument(
            'physical_config_file', default_value=PathJoinSubstitution([pkg, 'config', 'cefiro_params.yaml'])
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([pkg, 'rviz', 'cefiro.rviz'])
        ),
        publisher,
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui', condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='joint_state_publisher', condition=UnlessCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
    ])
