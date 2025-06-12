#!/usr/bin/env python3
"""
2-DOF Robot Arm Kinematics - UNLOCKED VERSION (No workspace limits)
Forward and Inverse kinematics for planar robot arm
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class ArmKinematics(Node):
    def __init__(self):
        super().__init__('arm_kinematics')
        
        # Arm segment lengths (cm)
        self.L1 = 15.0  # Base segment length
        self.L2 = 10.0  # End effector segment length
        
        # Subscribers
        self.position_cmd_sub = self.create_subscription(
            Point,
            'target_position',
            self.position_command_callback,
            10
        )
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        
        self.get_logger().info(f"🔓 Arm Kinematics initialized - UNLOCKED MODE (L1={self.L1}cm, L2={self.L2}cm)")
        self.get_logger().info("🔄 Workspace limits DISABLED - Any position allowed")
    
    def position_command_callback(self, msg):
        """Receive target position and calculate joint angles"""
        x = msg.x
        y = msg.y
        
        self.get_logger().info(f"🎯 Target position: ({x:.1f}, {y:.1f}) - UNLOCKED")
        
        # Calculate inverse kinematics - NO WORKSPACE LIMITS
        angles = self.inverse_kinematics_unlocked(x, y)
        
        if angles:
            joint1_deg, joint2_deg = angles
            
            # Convert to servo angles - NO LIMITS
            servo1_angle = joint1_deg  # Direct angle, no offset limitation
            servo2_angle = joint2_deg  # Direct angle, no offset limitation
            
            # Publish joint commands
            joint_cmd = Float64MultiArray()
            joint_cmd.data = [servo1_angle, servo2_angle]
            self.joint_cmd_pub.publish(joint_cmd)
            
            self.get_logger().info(f"🔄 Joint angles: {joint1_deg:.1f}°, {joint2_deg:.1f}° - UNLOCKED")
            self.get_logger().info(f"🔄 Servo angles: {servo1_angle:.1f}°, {servo2_angle:.1f}° - UNLOCKED")
        else:
            # Fallback: even if no solution, try alternative approach
            self.get_logger().warn("⚠️ Standard IK failed, trying alternative approach...")
            fallback_angles = self.alternative_kinematics(x, y)
            if fallback_angles:
                joint1_deg, joint2_deg = fallback_angles
                servo1_angle = joint1_deg
                servo2_angle = joint2_deg
                
                joint_cmd = Float64MultiArray()
                joint_cmd.data = [servo1_angle, servo2_angle]
                self.joint_cmd_pub.publish(joint_cmd)
                
                self.get_logger().info(f"🔄 Fallback angles: {joint1_deg:.1f}°, {joint2_deg:.1f}°")
    
    def inverse_kinematics_unlocked(self, x, y):
        """
        Calculate joint angles for target position (x, y) - NO WORKSPACE LIMITS
        Returns: (theta1, theta2) in degrees or None if mathematically impossible
        """
        # Distance from origin to target
        distance = math.sqrt(x*x + y*y)
        
        # UNLOCKED: Remove workspace limits - try to reach any position
        # Only check for mathematical impossibility (distance = 0)
        if distance < 0.1:  # Very close to origin, use small movement
            return (0.0, 0.0)
        
        # Calculate joint angles using law of cosines
        cos_theta2 = (distance*distance - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
        
        # For unreachable positions, clamp and find best approximation
        if cos_theta2 > 1:
            cos_theta2 = 1.0  # Fully extended
        elif cos_theta2 < -1:
            cos_theta2 = -1.0  # Fully folded
        
        theta2 = math.acos(cos_theta2)
        
        # Calculate theta1
        alpha = math.atan2(y, x)
        beta = math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        theta1 = alpha - beta
        
        # Convert to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        
        return (theta1_deg, theta2_deg)
    
    def alternative_kinematics(self, x, y):
        """
        Alternative kinematics approach for extreme positions
        """
        # Simple approach: point towards target and extend
        target_angle = math.degrees(math.atan2(y, x))
        extension_angle = 0  # Straight extension
        
        return (target_angle, extension_angle)
    
    def forward_kinematics(self, theta1_deg, theta2_deg):
        """
        Calculate end effector position from joint angles
        Returns: (x, y) position
        """
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        
        return (x, y)

def main(args=None):
    rclpy.init(args=args)
    kinematics = ArmKinematics()
    
    try:
        rclpy.spin(kinematics)
    except KeyboardInterrupt:
        pass
    finally:
        kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 