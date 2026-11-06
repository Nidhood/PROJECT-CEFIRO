#!/usr/bin/env python3
"""Spawn local scenery in order, then spawn CEFIRO after successful acknowledgements."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gz = FindPackageShare('cefiro_gz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_robot_after_models = LaunchConfiguration('spawn_robot_after_models')

    # These are local SDF files installed by the cefiro_gz package.
    desk_sdf = PathJoinSubstitution([
        pkg_gz, 'models', 'desk', 'sdf', 'desk.sdf'
    ])
    lamp_sdf = PathJoinSubstitution([
        pkg_gz, 'models', 'lamp', 'sdf', 'lamp.sdf'
    ])

    # Preserve the existing world-frame spawn poses.
    desk_position = [-0.19, -0.03, 0.54]
    desk_orientation = [0.0, 0.0, 0.0]
    lamp_position = [-0.68, 0.20, 0.84]
    lamp_orientation = [0.0, 0.0, 2.08]

    spawn_desk = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_desk',
        output='screen',
        arguments=[
            '-world', 'main_world',
            '-entity', 'office_desk',
            '-file', desk_sdf,
            '-allow_renaming', 'false',
            '-x', str(desk_position[0]),
            '-y', str(desk_position[1]),
            '-z', str(desk_position[2]),
            '-R', str(desk_orientation[0]),
            '-P', str(desk_orientation[1]),
            '-Y', str(desk_orientation[2]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_lamp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_lamp',
        output='screen',
        arguments=[
            '-world', 'main_world',
            '-entity', 'desk_lamp',
            '-file', lamp_sdf,
            '-allow_renaming', 'false',
            '-x', str(lamp_position[0]),
            '-y', str(lamp_position[1]),
            '-z', str(lamp_position[2]),
            '-R', str(lamp_orientation[0]),
            '-P', str(lamp_orientation[1]),
            '-Y', str(lamp_orientation[2]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_gz, 'launch', 'spawn_robot.launch.py'
        ])),
    )

    def on_desk_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg='[CEFIRO] ERROR: desk creation failed'),
                EmitEvent(event=Shutdown(reason='Could not create local desk model')),
            ]
        return [
            LogInfo(msg='[CEFIRO] Desk acknowledged; creating local lamp...'),
            spawn_lamp,
        ]

    def on_lamp_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg='[CEFIRO] ERROR: lamp creation failed'),
                EmitEvent(event=Shutdown(reason='Could not create local lamp model')),
            ]

        if spawn_robot_after_models.perform(context).lower() == 'true':
            return [
                LogInfo(msg='[CEFIRO] Desk + lamp acknowledged; spawning robot...'),
                spawn_robot,
            ]
        return [LogInfo(msg='[CEFIRO] Local desk + lamp creation completed')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time',
        ),
        DeclareLaunchArgument(
            'spawn_robot_after_models',
            default_value='false',
            choices=['true', 'false'],
            description='Spawn CEFIRO after successfully creating both local models',
        ),
        # Register handlers before launching the first creation process.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_desk,
            on_exit=on_desk_exit,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_lamp,
            on_exit=on_lamp_exit,
        )),
        LogInfo(msg='[CEFIRO] Creating local desk model...'),
        spawn_desk,
    ])
