#!/usr/bin/env python3
"""Launch existing C++ cefiro_gui with real-plant topic remapping."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cefiro_gui',
            executable='cefiro_gui_node',
            name='cefiro_hmi',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'imu_topic': '/real/imu/data',
                'magnetometer_topic': '/real/magnetometer/data',
                'barometer_topic': '/real/bmp581/air_pressure',
                'reference_topic': '/real/cefiro/control/attitude_reference',
                'enable_topic': '/real/cefiro/control/enable',
                'stop_topic': '/real/cefiro/control/emergency_stop',
                'mode_topic': '/real/cefiro/control/mode',
                'strategy_topic': '/real/cefiro/control/strategy',
                'status_topic': '/real/cefiro/control/status',
                'hardware_status_topic': '/real/cefiro/hardware/status',
                'pid_gains_topic': '/real/cefiro/control/pid_gains',
                'lqr_gain_topic': '/real/cefiro/control/lqr_gain',
                'motor_rpm_topic': '/real/cefiro_actuators/target_rpm',
                'motor_armed_topic': '/real/cefiro_actuators/armed',
                'motor_direction_topic': '/real/cefiro_actuators/motor_direction',
            }],
        ),
    ])
