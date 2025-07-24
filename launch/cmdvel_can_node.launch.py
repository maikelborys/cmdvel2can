#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmdvel2can',
            executable='cmdvel_can_node',
            name='cmdvel_can_node',
            output='screen',
        )
    ])
