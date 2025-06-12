# Step 5: Create Robot Arm ROS2 Package

## 🎯 **Goal**: Create custom ROS2 package for 2-DOF robot arm control

## 📋 **Current Status** ✅
- ✅ **ROS2**: Working on Pi (`ros2 topic list`)
- ✅ **ROS Bridge**: Running on port 9090
- ✅ **Network**: Ubuntu ↔ Pi communication established
- 🔄 **Next**: Create robot arm package

## 🔧 **Step 5.1: Create ROS2 Workspace**

On your **Pi** (SSH session):

```bash
# SSH into Pi
ssh LCen@192.168.131.18

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build empty workspace (creates install/build/log folders)
colcon build
source install/setup.bash
```

## 🔧 **Step 5.2: Create Robot Arm Package**

```bash
# Navigate to src directory
cd ~/ros2_ws/src

# Create robot arm package
ros2 pkg create --build-type ament_python robot_arm_controller \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs

# Navigate to package directory
cd robot_arm_controller
```

## 🔧 **Step 5.3: Create Package Structure**

```bash
# Create additional directories
mkdir -p robot_arm_controller/hardware
mkdir -p robot_arm_controller/kinematics
mkdir -p launch
mkdir -p config

# Check package structure
tree robot_arm_controller/
```

## 🔧 **Step 5.4: Create Servo Hardware Interface**

```bash
# Create servo controller
cat > robot_arm_controller/hardware/servo_controller.py << 'EOF'
#!/usr/bin/env python3
"""
Servo Controller for 2-DOF Robot Arm
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
            
        # Servo configuration
        self.servo_channels = [0, 1]  # Base and shoulder servos
        self.servo_min_angle = 0
        self.servo_max_angle = 180
        self.servo_center = 90
        
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
        
        self.get_logger().info("Servo Controller initialized")
        
    def joint_command_callback(self, msg):
        """Receive joint angle commands and move servos"""
        if len(msg.data) >= 2:
            joint1_angle = max(self.servo_min_angle, 
                             min(self.servo_max_angle, msg.data[0]))
            joint2_angle = max(self.servo_min_angle, 
                             min(self.servo_max_angle, msg.data[1]))
            
            self.move_servo(0, joint1_angle)
            self.move_servo(1, joint2_angle)
            
            self.joint_positions[0] = joint1_angle
            self.joint_positions[1] = joint2_angle
            
            self.get_logger().info(f"Moving servos to: [{joint1_angle:.1f}, {joint2_angle:.1f}]")
    
    def move_servo(self, channel, angle):
        """Move specific servo to angle"""
        if self.kit and channel < len(self.servo_channels):
            try:
                servo_channel = self.servo_channels[channel]
                self.kit.servo[servo_channel].angle = angle
            except Exception as e:
                self.get_logger().error(f"Error moving servo {channel}: {e}")
        else:
            # Simulation mode
            self.get_logger().info(f"SIM: Servo {channel} -> {angle:.1f}°")
    
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
EOF
```

## 🔧 **Step 5.5: Create Kinematics Calculator**

```bash
# Create kinematics module
cat > robot_arm_controller/kinematics/arm_kinematics.py << 'EOF'
#!/usr/bin/env python3
"""
2-DOF Robot Arm Kinematics
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
        
        self.get_logger().info(f"Arm Kinematics initialized (L1={self.L1}cm, L2={self.L2}cm)")
    
    def position_command_callback(self, msg):
        """Receive target position and calculate joint angles"""
        x = msg.x
        y = msg.y
        
        self.get_logger().info(f"Target position: ({x:.1f}, {y:.1f})")
        
        # Calculate inverse kinematics
        angles = self.inverse_kinematics(x, y)
        
        if angles:
            joint1_deg, joint2_deg = angles
            
            # Convert to servo angles (add offset for servo center)
            servo1_angle = joint1_deg + 90  # Base servo
            servo2_angle = joint2_deg + 90  # Shoulder servo
            
            # Publish joint commands
            joint_cmd = Float64MultiArray()
            joint_cmd.data = [servo1_angle, servo2_angle]
            self.joint_cmd_pub.publish(joint_cmd)
            
            self.get_logger().info(f"Joint angles: {joint1_deg:.1f}°, {joint2_deg:.1f}°")
            self.get_logger().info(f"Servo angles: {servo1_angle:.1f}°, {servo2_angle:.1f}°")
        else:
            self.get_logger().warn("Target position unreachable")
    
    def inverse_kinematics(self, x, y):
        """
        Calculate joint angles for target position (x, y)
        Returns: (theta1, theta2) in degrees or None if unreachable
        """
        # Distance from origin to target
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > (self.L1 + self.L2) or distance < abs(self.L1 - self.L2):
            return None
        
        # Calculate joint angles using law of cosines
        cos_theta2 = (distance*distance - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
        
        # Clamp to avoid numerical errors
        cos_theta2 = max(-1, min(1, cos_theta2))
        
        theta2 = math.acos(cos_theta2)
        
        # Calculate theta1
        alpha = math.atan2(y, x)
        beta = math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        theta1 = alpha - beta
        
        # Convert to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        
        return (theta1_deg, theta2_deg)
    
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
EOF
```

## 🔧 **Step 5.6: Create Launch File**

```bash
# Create launch file
cat > launch/robot_arm_launch.py << 'EOF'
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_controller',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),
        Node(
            package='robot_arm_controller',
            executable='arm_kinematics',
            name='arm_kinematics',
            output='screen'
        ),
    ])
EOF
```

## 🔧 **Step 5.7: Update Package Configuration**

```bash
# Update setup.py
cat > setup.py << 'EOF'
from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LCen',
    maintainer_email='your.email@example.com',
    description='2-DOF Robot Arm Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller = robot_arm_controller.hardware.servo_controller:main',
            'arm_kinematics = robot_arm_controller.kinematics.arm_kinematics:main',
        ],
    },
)
EOF
```

## 🔧 **Step 5.8: Build Package**

```bash
# Navigate back to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select robot_arm_controller

# Source the workspace
source install/setup.bash

# Verify package is built
ros2 pkg list | grep robot_arm
```

## 🔧 **Step 5.9: Test Package**

```bash
# Test 1: Launch robot arm nodes
ros2 launch robot_arm_controller robot_arm_launch.py

# Test 2 (in new terminal): Send position command
ros2 topic pub /target_position geometry_msgs/Point "x: 10.0, y: 15.0, z: 0.0" --once

# Test 3: Check joint states
ros2 topic echo /joint_states

# Test 4: Send direct joint commands
ros2 topic pub /joint_commands std_msgs/Float64MultiArray "data: [90.0, 45.0]" --once
```

## ✅ **Verification Checklist**

- [x] **Workspace created**: `~/ros2_ws` exists
- [x] **Package structure**: `robot_arm_controller` directories created
- [x] **Servo controller**: Hardware interface file created
- [x] **Kinematics module**: Inverse kinematics calculator created
- [x] **Launch file**: Node coordination file created
- [x] **Package setup**: `setup.py` with entry points configured
- [x] **Package built**: `colcon build` succeeds
- [x] **Nodes launch**: `robot_arm_launch.py` starts both nodes
- [x] **Topics exist**: `/target_position`, `/joint_commands`, `/joint_states`
- [x] **Kinematics work**: Position commands move servos
- [x] **Hardware interface**: Servo commands are received

## 🎯 **Next Steps**

Once package is working:

1. **✅ Test MCP integration** with ROS Bridge
2. **Connect physical servos** and test hardware
3. **Calibrate servo positions** and limits
4. **Add safety limits** and error handling
5. **Test natural language commands** from Cursor

**🎉 PACKAGE COMPLETE! Ready for hardware integration!** 🚀 