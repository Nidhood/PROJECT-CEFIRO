#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Gazebo launch path:
    gz_launch_path = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ])
    
    # World file path:
    world_file_path = PathJoinSubstitution([
        FindPackageShare("cefiro_gz"),
        "worlds",
        "main_world.sdf"
    ])

    # GUI config file path:
    gui_config_path = PathJoinSubstitution([
        FindPackageShare("cefiro_gz"),
        "config",
        "scenery.config"
    ])
    
    cefiro_gz_prefix = get_package_prefix('cefiro_gz')
    cefiro_sensors_prefix = get_package_prefix('cefiro_sensors')

    gazebo_models_path = os.path.join(
        get_package_share_directory('cefiro_gz'),
        'models'
    )

    cefiro_gz_share = os.path.join(
        cefiro_gz_prefix,
        'share'
    )

    cefiro_sensors_share = os.path.join(
        cefiro_sensors_prefix,
        'share'
    )

    # Environment variables configuration:
    env_vars = [

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join([
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                cefiro_gz_share,
                cefiro_sensors_share,
                gazebo_models_path,
            ])
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=':'.join([
                os.environ.get('GAZEBO_MODEL_PATH', ''),
                cefiro_gz_share,
                cefiro_sensors_share,
                gazebo_models_path,
            ])
        ),

        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=':'.join([
                os.environ.get('GAZEBO_PLUGIN_PATH', ''),
                os.path.join(cefiro_gz_prefix, 'lib'),
            ])
        ),
    ]
    
    # Gazebo launch description:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                '-r -v 4 ',
                world_file_path,
                ' --gui-config ',
                gui_config_path
            ]
        }.items(),
    )
    
    return LaunchDescription(
        env_vars + [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation clock'
            ),
            gazebo
        ]
    )