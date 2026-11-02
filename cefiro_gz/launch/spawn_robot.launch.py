#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Assign the entity name:
    entity_name = "cefiro"

    # Define the starting position of the robot in the Gazebo simulation:
    position = [0.09, -0.11, 0.805]

    # Define the orientation of the robot in the Gazebo simulation:
    orientation = [0.0, 0.0, 1.07]

    # Spawn the robot model in the Gazebo simulation:
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-world', 'main_world',
            '-entity', entity_name,
            '-topic', 'robot_description',
            '-allow_renaming', 'true',
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2])
        ]
    )
    
    return LaunchDescription([
        spawn_robot
    ])