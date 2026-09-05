#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    publish_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cefiro_sensors'),
                'launch',
                'publish_bmp581_urdf.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cefiro_sensors'),
        'rviz',
        'bmp581_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui'
        ),
        publish_description_launch,
        rviz_node
    ])