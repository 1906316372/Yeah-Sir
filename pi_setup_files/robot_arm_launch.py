#!/usr/bin/env python3
"""
Launch file for 2-DOF Robot Arm system
Starts all necessary nodes for robot arm operation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Start joint state publisher
        Node(
            package='robot_arm_controller',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Start motor controller
        Node(
            package='robot_arm_controller', 
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Start cartesian controller
        Node(
            package='robot_arm_controller',
            executable='cartesian_controller', 
            name='cartesian_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Start ROS Bridge Server
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
    ]) 