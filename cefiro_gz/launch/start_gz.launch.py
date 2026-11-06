#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_sensors = LaunchConfiguration('enable_sensors')
    pkg_gz = FindPackageShare('cefiro_gz')
    pkg_control = FindPackageShare('cefiro_control')
    pkg_description = FindPackageShare('cefiro_description')

    gazebo_launch = PathJoinSubstitution([
        pkg_gz, 'launch', 'spawn_world.launch.py'
    ])

    urdf_launch = PathJoinSubstitution([
        pkg_control, 'launch', 'publish_cefiro_control_urdf.launch.py'
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

    # Dedicated simulation-only controller. Do not spawn the legacy
    # ros2_control plugin, which is not wired to the standalone actuator node.
    balance = Node(
        package='cefiro_control',
        executable='cefiro_balance_node.py',
        name='cefiro_balance',
        parameters=[PathJoinSubstitution([
            pkg_control, 'config', 'balance_sim.yaml'
        ]), {'use_sim_time': use_sim_time}],
        output='screen',
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

    # Gazebo system plugin from the compiled local cefiro_gz install/lib.
    rotor_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', [
            PathJoinSubstitution([FindPackagePrefix('cefiro_gz'), 'lib']),
            ':', EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
        ],
    )
    actuator_node = Node(
        package='cefiro_actuators',
        executable='cefiro_actuators_node',
        name='cefiro_actuators',
        parameters=[PathJoinSubstitution([
            FindPackageShare('cefiro_actuators'), 'config', 'actuators.yaml'
        ]), {'use_sim_time': use_sim_time}],
        output='screen',
    )

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
        rotor_plugin_path,
        world,
        actuator_node,
        description,
        balance,
        bridge,
        wait_for_server,
    ])
