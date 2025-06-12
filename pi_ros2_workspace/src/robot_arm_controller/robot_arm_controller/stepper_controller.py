#!/usr/bin/env python3
"""
Stepper Motor Controller for 2-DOF Robot Arm
Controls NEMA 11 steppers with planetary gearboxes via GPIO step/direction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import threading

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available, using simulation mode")

class StepperController(Node):
    def __init__(self):
        super().__init__('stepper_controller')
        
        # Motor specifications
        self.STEPS_PER_REV = 200  # 1.8° stepper
        self.GEAR_RATIO = 13.73   # Planetary gearbox
        self.MICROSTEPS = 16      # 1/16 microstepping
        self.TOTAL_STEPS_PER_REV = int(self.STEPS_PER_REV * self.GEAR_RATIO * self.MICROSTEPS)
        
        # GPIO pin assignments
        self.motor_pins = {
            0: {'step': 18, 'dir': 19, 'enable': 20},  # Base motor
            1: {'step': 21, 'dir': 22, 'enable': 23}   # Shoulder motor
        }
        
        # Motor state
        self.current_positions = [0, 0]  # Current position in steps
        self.target_positions = [0, 0]   # Target position in steps
        self.is_moving = [False, False]  # Movement status
        
        # Movement parameters
        self.max_speed = 2000      # Steps per second
        self.acceleration = 5000   # Steps per second²
        self.min_step_delay = 1.0 / self.max_speed
        
        # Initialize GPIO
        if GPIO_AVAILABLE:
            self.init_gpio()
        else:
            self.get_logger().warn("GPIO not available - running in simulation mode")
        
        # ROS2 interfaces
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info(f"Stepper Controller initialized")
        self.get_logger().info(f"Resolution: {self.TOTAL_STEPS_PER_REV} steps/revolution")
        self.get_logger().info(f"Precision: {360.0/self.TOTAL_STEPS_PER_REV:.4f}° per step")
        
    def init_gpio(self):
        """Initialize GPIO pins for stepper control"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            for motor_id, pins in self.motor_pins.items():
                # Setup pins
                GPIO.setup(pins['step'], GPIO.OUT)
                GPIO.setup(pins['dir'], GPIO.OUT)
                GPIO.setup(pins['enable'], GPIO.OUT)
                
                # Initialize states
                GPIO.output(pins['step'], GPIO.LOW)
                GPIO.output(pins['dir'], GPIO.LOW)
                GPIO.output(pins['enable'], GPIO.LOW)  # Enable motors (active low)
                
            self.get_logger().info("GPIO initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            global GPIO_AVAILABLE
            GPIO_AVAILABLE = False
    
    def degrees_to_steps(self, degrees):
        """Convert degrees to step count"""
        return int((degrees / 360.0) * self.TOTAL_STEPS_PER_REV)
    
    def steps_to_degrees(self, steps):
        """Convert step count to degrees"""
        return (steps / self.TOTAL_STEPS_PER_REV) * 360.0
    
    def joint_command_callback(self, msg):
        """Receive joint angle commands and move steppers"""
        if len(msg.data) >= 2:
            # Convert servo angles to absolute positions
            # Original servo range: 0-180°, center at 90°
            # Convert to stepper absolute positions
            target_angle_1 = msg.data[0] - 90.0  # Convert servo angle to relative angle
            target_angle_2 = msg.data[1] - 90.0
            
            target_steps_1 = self.degrees_to_steps(target_angle_1)
            target_steps_2 = self.degrees_to_steps(target_angle_2)
            
            self.get_logger().info(f"Target angles: [{target_angle_1:.1f}°, {target_angle_2:.1f}°]")
            self.get_logger().info(f"Target steps: [{target_steps_1}, {target_steps_2}]")
            
            # Move motors to target positions
            self.move_motor_to_position(0, target_steps_1)
            self.move_motor_to_position(1, target_steps_2)
    
    def move_motor_to_position(self, motor_id, target_steps):
        """Move specific motor to target position"""
        if motor_id not in [0, 1]:
            return
            
        current_steps = self.current_positions[motor_id]
        steps_to_move = target_steps - current_steps
        
        if steps_to_move == 0:
            return
            
        self.target_positions[motor_id] = target_steps
        
        if GPIO_AVAILABLE:
            # Start movement in separate thread
            thread = threading.Thread(
                target=self._execute_movement,
                args=(motor_id, steps_to_move)
            )
            thread.daemon = True
            thread.start()
        else:
            # Simulation mode
            self.current_positions[motor_id] = target_steps
            angle = self.steps_to_degrees(target_steps)
            self.get_logger().info(f"SIM: Motor {motor_id} -> {angle:.1f}° ({target_steps} steps)")
    
    def _execute_movement(self, motor_id, steps_to_move):
        """Execute stepper movement with acceleration profile"""
        if not GPIO_AVAILABLE:
            return
            
        self.is_moving[motor_id] = True
        pins = self.motor_pins[motor_id]
        
        # Set direction
        direction = 1 if steps_to_move > 0 else -1
        GPIO.output(pins['dir'], GPIO.HIGH if direction > 0 else GPIO.LOW)
        
        abs_steps = abs(steps_to_move)
        
        # Simple trapezoidal acceleration profile
        accel_steps = min(abs_steps // 4, 500)  # Acceleration phase
        decel_steps = accel_steps                # Deceleration phase
        const_steps = abs_steps - accel_steps - decel_steps
        
        try:
            step_count = 0
            
            # Acceleration phase
            for i in range(accel_steps):
                speed_factor = (i + 1) / accel_steps
                delay = self.min_step_delay / speed_factor
                self._make_step(pins['step'], delay)
                step_count += 1
                self.current_positions[motor_id] += direction
            
            # Constant speed phase
            for i in range(const_steps):
                self._make_step(pins['step'], self.min_step_delay)
                step_count += 1
                self.current_positions[motor_id] += direction
            
            # Deceleration phase
            for i in range(decel_steps):
                speed_factor = (decel_steps - i) / decel_steps
                delay = self.min_step_delay / speed_factor
                self._make_step(pins['step'], delay)
                step_count += 1
                self.current_positions[motor_id] += direction
                
            angle = self.steps_to_degrees(self.current_positions[motor_id])
            self.get_logger().info(f"Motor {motor_id} completed: {angle:.1f}° ({self.current_positions[motor_id]} steps)")
            
        except Exception as e:
            self.get_logger().error(f"Movement error on motor {motor_id}: {e}")
        finally:
            self.is_moving[motor_id] = False
    
    def _make_step(self, step_pin, delay):
        """Generate single step pulse"""
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.000001)  # 1µs pulse width
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(delay)
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2']
        
        # Convert current step positions to radians
        angle1_deg = self.steps_to_degrees(self.current_positions[0])
        angle2_deg = self.steps_to_degrees(self.current_positions[1])
        
        joint_state.position = [
            angle1_deg * 3.14159 / 180.0,  # Convert to radians
            angle2_deg * 3.14159 / 180.0
        ]
        joint_state.velocity = [0.0, 0.0]  # Could add velocity calculation
        joint_state.effort = [0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)
    
    def home_motors(self):
        """Move both motors to home position (0 steps)"""
        self.get_logger().info("Homing motors...")
        self.move_motor_to_position(0, 0)
        self.move_motor_to_position(1, 0)
    
    def emergency_stop(self):
        """Emergency stop - disable all motors"""
        if GPIO_AVAILABLE:
            for pins in self.motor_pins.values():
                GPIO.output(pins['enable'], GPIO.HIGH)  # Disable motors
        self.get_logger().warn("EMERGENCY STOP - Motors disabled")
    
    def __del__(self):
        """Cleanup GPIO on destruction"""
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    stepper_controller = StepperController()
    
    try:
        rclpy.spin(stepper_controller)
    except KeyboardInterrupt:
        stepper_controller.get_logger().info("Shutting down stepper controller...")
    finally:
        stepper_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 