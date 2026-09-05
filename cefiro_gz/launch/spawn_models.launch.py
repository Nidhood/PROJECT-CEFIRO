#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments:
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Find package share directory:
    pkg_gz = FindPackageShare("cefiro_gz")

    # Path to desk SDF model:
    desk_sdf_path = PathJoinSubstitution([
        pkg_gz,
        "models",
        "desk",
        "sdf",
        "desk.sdf"
    ])

    # Path to lamp SDF model:
    lamp_sdf_path = PathJoinSubstitution([
        pkg_gz,
        "models",
        "lamp",
        "sdf",
        "lamp.sdf"
    ])

    # Desk spawn configuration:
    desk_name = "office_desk"
    desk_position = [-0.19, -0.03, 0.54]       # X, Y, Z (meters)
    desk_orientation = [0.0, 0.0, 0.0]         # Roll, Pitch, Yaw (radians)

    # Lamp spawn configuration:
    lamp_name = "desk_lamp"
    lamp_position = [-0.68, 0.20, 0.84]        # X, Y, Z (meters)
    lamp_orientation = [0.0, 0.0, 2.08]        # Roll, Pitch, Yaw (radians)

    # Spawn desk model in Gazebo:
    spawn_desk = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_desk',
        output='screen',
        arguments=[
            '-entity', desk_name,
            '-file', desk_sdf_path,
            '-allow_renaming', 'true',
            '-x', str(desk_position[0]),
            '-y', str(desk_position[1]),
            '-z', str(desk_position[2]),
            '-R', str(desk_orientation[0]),
            '-P', str(desk_orientation[1]),
            '-Y', str(desk_orientation[2]),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn lamp model in Gazebo:
    spawn_lamp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_lamp',
        output='screen',
        arguments=[
            '-entity', lamp_name,
            '-file', lamp_sdf_path,
            '-allow_renaming', 'true',
            '-x', str(lamp_position[0]),
            '-y', str(lamp_position[1]),
            '-z', str(lamp_position[2]),
            '-R', str(lamp_orientation[0]),
            '-P', str(lamp_orientation[1]),
            '-Y', str(lamp_orientation[2]),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        spawn_desk,
        spawn_lamp,
    ])