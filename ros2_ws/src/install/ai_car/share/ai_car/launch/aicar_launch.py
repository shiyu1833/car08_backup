#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_car',
            executable='aicar_base',
            ),
        Node(
            package='ai_car',
            executable='aicar_imu',
            ),
        Node(
            package='ai_car',
            executable='aicar_odom',
            ),  
    ])