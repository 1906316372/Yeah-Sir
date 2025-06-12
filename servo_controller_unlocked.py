#!/usr/bin/env python3
"""
Servo Controller for 2-DOF Robot Arm - UNLOCKED VERSION (360° rotation)
Interfaces with PCA9685 PWM driver via I2C
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

try:
    from adafruit_servokit import ServoKit
    SERVO_AVAILABLE = True
except ImportError:
    SERVO_AVAILABLE = False
    print("Warning: adafruit-servokit not available, using simulation mode")

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Initialize servo kit (16-channel PCA9685)
        if SERVO_AVAILABLE:
            try:
                self.kit = ServoKit(channels=16)
                self.get_logger().info("PCA9685 ServoKit initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize ServoKit: {e}")
                self.kit = None
        else:
            self.kit = None
            
        # Servo configuration - UNLOCKED (360° rotation)
        self.servo_channels = [0, 1]  # Base and shoulder servos
        self.servo_min_angle = -999   # UNLOCKED: Allow unlimited negative rotation
        self.servo_max_angle = 999    # UNLOCKED: Allow unlimited positive rotation
        self.servo_center = 0         # Center at 0° instead of 90°
        
        # Current joint positions (degrees)
        self.joint_positions = [self.servo_center, self.servo_center]
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info("🔓 Servo Controller initialized - UNLOCKED MODE (360° rotation enabled)")
        self.get_logger().info(f"Angle range: UNLIMITED ({self.servo_min_angle}° to {self.servo_max_angle}°)")
        
    def joint_command_callback(self, msg):
        """Receive joint angle commands and move servos - NO ANGLE LIMITS"""
        if len(msg.data) >= 2:
            # UNLOCKED: No angle clamping - accept any angle
            joint1_angle = msg.data[0]  # No limits!
            joint2_angle = msg.data[1]  # No limits!
            
            self.move_servo(0, joint1_angle)
            self.move_servo(1, joint2_angle)
            
            self.joint_positions[0] = joint1_angle
            self.joint_positions[1] = joint2_angle
            
            self.get_logger().info(f"🔄 Moving servos to: [{joint1_angle:.1f}°, {joint2_angle:.1f}°] - UNLOCKED")
    
    def move_servo(self, channel, angle):
        """Move specific servo to angle - handles 360° rotation"""
        if self.kit and channel < len(self.servo_channels):
            try:
                servo_channel = self.servo_channels[channel]
                # For continuous rotation servos or steppers, pass angle directly
                # If using standard servos, map to 0-180 hardware range
                mapped_angle = angle % 360
                if mapped_angle > 180:
                    mapped_angle = mapped_angle - 360  # Keep in -180 to 180 range
                
                # Map to servo range (assuming 0-180 hardware range)
                servo_angle = mapped_angle + 90  # Shift to 0-180 range
                servo_angle = max(0, min(180, servo_angle))  # Hardware safety clamp only
                
                self.kit.servo[servo_channel].angle = servo_angle
                self.get_logger().debug(f"Servo {channel}: {angle:.1f}° -> hardware {servo_angle:.1f}°")
            except Exception as e:
                self.get_logger().error(f"Error moving servo {channel}: {e}")
        else:
            # Simulation mode
            self.get_logger().info(f"🔄 SIM: Servo {channel} -> {angle:.1f}° (UNLOCKED)")
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2']
        joint_state.position = [
            self.joint_positions[0] * 3.14159 / 180.0,  # Convert to radians
            self.joint_positions[1] * 3.14159 / 180.0
        ]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 