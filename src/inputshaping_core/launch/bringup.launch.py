"""Launch the inputshaping bench: MPU bridge + experiment node + GUI."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = os.environ.get(
        'INPUTSHAPING_SERIAL_PORT', '/dev/ttyACM0')
    gui_port = int(os.environ.get('INPUTSHAPING_GUI_PORT', 8080))

    return LaunchDescription([
        Node(
            package='inputshaping_core',
            executable='mpu_serial_node',
            name='mpu_serial_node',
            output='screen',
            parameters=[{
                'port': serial_port,
                'topic': '/imu/data_raw',
                'frame_id': 'imu_link',
            }],
        ),
        Node(
            package='inputshaping_core',
            executable='experiment_node',
            name='inputshaping_experiment_node',
            output='screen',
            parameters=[{
                'cmd_vel_raw_topic': '/cmd_vel_raw',
                'cmd_vel_shaped_topic': '/cmd_vel_shaped',
                'imu_topic': '/imu/data_raw',
                'gui_host': '0.0.0.0',
                'gui_port': gui_port,
                'data_dir': '/workspace/data',
                'v_max_default': 0.25,
                'a_max_default': 0.5,
                'v_max_limit': 0.5,
                'a_max_limit': 1.0,
            }],
        ),
    ])
