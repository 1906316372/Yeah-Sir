#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_controller',
            executable='stepper_controller',
            name='stepper_controller',
            output='screen'
        ),
        Node(
            package='robot_arm_controller',
            executable='arm_kinematics',
            name='arm_kinematics',
            output='screen'
        ),
    ])
