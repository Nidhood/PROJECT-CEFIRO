#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Assign the entity name:
    entity_name = "bmp581"

    # Define the starting position of the robot in the Gazebo simulation:
    position = [0.29, -0.04, 0.91]

    # Define the orientation of the robot in the Gazebo simulation:
    orientation = [0.0, 0.0, 1.0]

    # Spawn the robot model in the Gazebo simulation:
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-topic', 'robot_description',
            '-allow_renaming', 'true',
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
            '--initial-joint-positions', 'arm_link_joint:=-1.508'
        ]
    )
    
    return LaunchDescription([
        spawn_robot
    ])