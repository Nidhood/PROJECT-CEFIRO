#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration( 'use_sim_time' )
    use_joint_state_gui = LaunchConfiguration( 'use_joint_state_gui' )
    
    publish_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare( 'cefiro_description' ),
                'launch',
                'publish_cefiro_urdf.launch.py'
            ])
        ),
    launch_arguments = {
        'use_sim_time': use_sim_time
    }.items()
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',

        condition=IfCondition(use_joint_state_gui),

        parameters=[{
            'use_sim_time': use_sim_time
        }],

        output='screen'
    )
    
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cefiro_description'),
        'rviz',
        'cefiro.rviz'
    ])
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',

        arguments=[
            '-d',
            rviz_config_file
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
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
        joint_state_publisher_gui_node,
        rviz_node
    ])