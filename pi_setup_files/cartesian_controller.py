#!/usr/bin/env python3
"""
Cartesian Controller for 2-DOF Robot Arm
Handles high-level movement commands from MCP server
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import json
import time

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/arm_status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/arm_commands', self.command_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Current state
        self.current_position = {'x': 0.0, 'y': 0.15}  # Start position
        self.current_joints = {'joint1': 0.0, 'joint2': 0.0}
        
        # Arm parameters (must match motor_controller)
        self.arm_length_1 = 0.15  # 15cm
        self.arm_length_2 = 0.10  # 10cm
        
        # Movement parameters
        self.max_speed = 0.05  # 5cm/s max speed
        self.position_tolerance = 0.005  # 5mm tolerance
        
        self.get_logger().info('Cartesian Controller started')
        self.publish_status("Ready for commands")

    def command_callback(self, msg):
        """Handle natural language movement commands"""
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f'Received command: {command}')
            
            command_type = command.get('type', '')
            
            if command_type == 'move_relative':
                self.handle_relative_move(command)
            elif command_type == 'move_absolute':
                self.handle_absolute_move(command)
            elif command_type == 'move_joint':
                self.handle_joint_move(command)
            elif command_type == 'home':
                self.handle_home()
            elif command_type == 'stop':
                self.handle_stop()
            else:
                self.get_logger().warn(f'Unknown command type: {command_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')

    def handle_relative_move(self, command):
        """Handle relative movement commands like 'move 2cm along x direction'"""
        dx = command.get('dx', 0.0) / 100.0  # Convert cm to meters
        dy = command.get('dy', 0.0) / 100.0
        
        self.get_logger().info(f'Relative move: dx={dx:.3f}m, dy={dy:.3f}m')
        
        # Create and send Twist message
        twist = Twist()
        twist.linear.x = dx
        twist.linear.y = dy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.twist_pub.publish(twist)
        self.publish_status(f"Moving {dx*100:.1f}cm in X, {dy*100:.1f}cm in Y")

    def handle_absolute_move(self, command):
        """Handle absolute position commands"""
        target_x = command.get('x', 0.0) / 100.0  # Convert cm to meters
        target_y = command.get('y', 0.0) / 100.0
        
        # Calculate relative movement needed
        dx = target_x - self.current_position['x']
        dy = target_y - self.current_position['y']
        
        self.get_logger().info(f'Absolute move to: x={target_x:.3f}m, y={target_y:.3f}m')
        
        # Send as relative movement
        twist = Twist()
        twist.linear.x = dx
        twist.linear.y = dy
        twist.linear.z = 0.0
        
        self.twist_pub.publish(twist)
        self.publish_status(f"Moving to position ({target_x*100:.1f}, {target_y*100:.1f})cm")

    def handle_joint_move(self, command):
        """Handle direct joint angle commands"""
        joint1_deg = command.get('joint1', 0.0)
        joint2_deg = command.get('joint2', 0.0)
        
        # Convert degrees to radians
        joint1_rad = math.radians(joint1_deg)
        joint2_rad = math.radians(joint2_deg)
        
        self.get_logger().info(f'Joint move: J1={joint1_deg:.1f}°, J2={joint2_deg:.1f}°')
        
        # Create joint command
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = ['joint1', 'joint2']
        joint_cmd.position = [joint1_rad, joint2_rad]
        joint_cmd.velocity = [0.0, 0.0]
        joint_cmd.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_cmd)
        self.publish_status(f"Moving joints to J1={joint1_deg:.1f}°, J2={joint2_deg:.1f}°")

    def handle_home(self):
        """Move arm to home position"""
        self.get_logger().info('Moving to home position')
        
        # Home position (straight up)
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = ['joint1', 'joint2']
        joint_cmd.position = [0.0, 0.0]  # Both joints at 0 degrees
        joint_cmd.velocity = [0.0, 0.0]
        joint_cmd.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_cmd)
        self.publish_status("Moving to home position")

    def handle_stop(self):
        """Stop all movement"""
        self.get_logger().info('Stopping arm movement')
        
        # Send zero velocity
        twist = Twist()
        self.twist_pub.publish(twist)
        self.publish_status("Arm stopped")

    def joint_state_callback(self, msg):
        """Update current joint states"""
        if len(msg.name) >= 2 and len(msg.position) >= 2:
            for i, name in enumerate(msg.name):
                if name in self.current_joints:
                    self.current_joints[name] = msg.position[i]
            
            # Calculate current Cartesian position using forward kinematics
            self.update_cartesian_position()

    def update_cartesian_position(self):
        """Calculate current Cartesian position from joint angles"""
        j1 = self.current_joints['joint1']
        j2 = self.current_joints['joint2']
        
        # Forward kinematics for 2-DOF planar arm
        l1, l2 = self.arm_length_1, self.arm_length_2
        
        x = l1 * math.cos(j1) + l2 * math.cos(j1 + j2)
        y = l1 * math.sin(j1) + l2 * math.sin(j1 + j2)
        
        self.current_position['x'] = x
        self.current_position['y'] = y

    def publish_status(self, message):
        """Publish status message"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

    def get_current_position(self):
        """Get current position in cm for easy reading"""
        return {
            'x_cm': self.current_position['x'] * 100,
            'y_cm': self.current_position['y'] * 100,
            'joint1_deg': math.degrees(self.current_joints['joint1']),
            'joint2_deg': math.degrees(self.current_joints['joint2'])
        }

def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cartesian Controller shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 