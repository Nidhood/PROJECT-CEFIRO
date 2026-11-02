#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_sensors = LaunchConfiguration('enable_sensors')
    pkg_gz = FindPackageShare('cefiro_gz')
    pkg_description = FindPackageShare('cefiro_description')

    gazebo_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'spawn_world.launch.py'
    ])
    urdf_launch = PathJoinSubstitution([
        pkg_description, 'launch', 'publish_cefiro_urdf.launch.py'
    ])
    models_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'spawn_models.launch.py'
    ])
    bridge_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'gz_bridge.launch.py'
    ])

    # Start Gazebo and independent ROS nodes immediately. Scenery begins
    # only when the local world creation service becomes available.
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch),
        launch_arguments={'use_sim_time': use_sim_time, 'enable_sensors': enable_sensors}.items()
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch)
    )

    # spawn_models creates desk -> lamp -> robot, each triggered only when
    # the preceding local ros_gz_sim/create command finishes successfully.
    scenery_then_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(models_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'spawn_robot_after_models': 'true',
        }.items()
    )

    wait_for_server = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                pkg_gz, 'scripts', 'wait_for_gazebo_ready.py'
            ]),
            '--world', 'main_world',
            '--timeout', '120',
        ],
        name='wait_for_gazebo',
        output='screen',
    )

    def on_gazebo_ready(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg='[CEFIRO] Gazebo did not become ready'),
                EmitEvent(event=Shutdown(
                    reason='Local Gazebo create service unavailable'
                )),
            ]
        return [
            LogInfo(msg='[CEFIRO] Gazebo ready; spawning scenery...'),
            scenery_then_robot,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time'
        ),
        DeclareLaunchArgument(
            'enable_sensors', default_value='true', choices=['true', 'false'],
            description='Enable the onboard FMU sensors',
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_server,
            on_exit=on_gazebo_ready,
        )),
        world,
        description,
        bridge,
        wait_for_server,
    ])
