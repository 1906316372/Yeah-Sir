# Raspberry Pi 4B Robot Arm Setup with ROS2

## 🎯 **Project Goal**
Create a 2-DOF robot arm controlled via natural language commands through Cursor → ROS MCP Server → ROS2 → Raspberry Pi

## 🏗️ **System Architecture**

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Cursor    │───▶│ ROS MCP     │───▶│ ROS Bridge  │───▶│ Raspberry   │
│ (Natural    │    │ Server      │    │ WebSocket   │    │ Pi 4B       │
│ Language)   │    │ (Ubuntu)    │    │             │    │ (ROS2)      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                                                                │
                                                                ▼
                                                    ┌─────────────────────┐
                                                    │   Motor Driver      │
                                                    │   + 2 Motors        │
                                                    │   + Robot Arm       │
                                                    └─────────────────────┘
```

## 📋 **Hardware Requirements**

### Raspberry Pi Setup:
- **Raspberry Pi 4B** (4GB+ RAM recommended)
- **MicroSD Card** (32GB+ Class 10)
- **Power Supply** (5V 3A)
- **Ethernet/WiFi** connection

### Robot Arm Hardware:
- **2x Encoderless Motors** (servo motors or stepper motors recommended)
- **Motor Driver Board** (e.g., L298N, PCA9685, or dedicated servo driver)
- **2x Wooden Bars** (for arm segments)
- **Mechanical joints** (bearings/servo horns)
- **Power Supply** for motors
- **Jumper wires** and breadboard

## 🔧 **Step 1: Raspberry Pi OS & ROS2 Setup**

### Install Raspberry Pi OS
```bash
# Flash Raspberry Pi OS (64-bit) to SD card
# Enable SSH and WiFi during setup

# First boot - update system
sudo apt update && sudo apt upgrade -y
sudo reboot
```

### Install ROS2 Humble (recommended for Pi)
```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y python3-argcomplete

# Install development tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Setup ROS2 Environment
```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc

# Test ROS2 installation
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
# Should see communication between nodes
```

## 🌉 **Step 2: Install ROS Bridge**

```bash
# Install rosbridge for communication with MCP server
sudo apt install -y ros-humble-rosbridge-server
sudo apt install -y ros-humble-tf2-web-republisher

# Test rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# Should start websocket server on port 9090
```

## 🦾 **Step 3: Create Robot Arm ROS2 Package**

```bash
# Create workspace
mkdir -p ~/robot_arm_ws/src
cd ~/robot_arm_ws/src

# Create package
ros2 pkg create --build-type ament_python robot_arm_controller --dependencies rclpy std_msgs geometry_msgs sensor_msgs

cd ~/robot_arm_ws
colcon build
source install/setup.bash
```

## 📝 **Step 4: Robot Arm Controller Code**

Let me create the ROS2 nodes for your robot arm:

### Joint State Publisher
```python
# ~/robot_arm_ws/src/robot_arm_controller/robot_arm_controller/joint_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # 10Hz
        
        # Current joint positions (in radians)
        self.joint1_pos = 0.0  # Base rotation
        self.joint2_pos = 0.0  # Arm extension
        
        self.get_logger().info('Joint State Publisher started')

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        joint_state.name = ['joint1', 'joint2']
        joint_state.position = [self.joint1_pos, self.joint2_pos]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)

    def update_joint_positions(self, j1_pos, j2_pos):
        """Update joint positions from motor feedback"""
        self.joint1_pos = j1_pos
        self.joint2_pos = j2_pos

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Motor Controller
```python
# ~/robot_arm_ws/src/robot_arm_controller/robot_arm_controller/motor_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

# Import your motor driver library here
# For example, if using servo motors:
try:
    import RPi.GPIO as GPIO
    from adafruit_servokit import ServoKit
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False
    print("Warning: Hardware libraries not available. Running in simulation mode.")

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Subscribe to movement commands
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        
        # Subscribe to joint commands
        self.joint_sub = self.create_subscription(
            JointState, '/joint_commands', self.joint_callback, 10)
        
        # Hardware setup
        self.setup_hardware()
        
        # Robot arm parameters
        self.arm_length_1 = 0.15  # 15cm first segment
        self.arm_length_2 = 0.10  # 10cm second segment
        
        # Current positions
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_angle1 = 0.0  # Base joint
        self.current_angle2 = 0.0  # Second joint
        
        self.get_logger().info('Motor Controller started')

    def setup_hardware(self):
        """Initialize motor hardware"""
        if HAS_HARDWARE:
            try:
                # For servo motors (adjust based on your motor driver)
                self.kit = ServoKit(channels=16)
                self.motor1 = self.kit.servo[0]  # Base rotation
                self.motor2 = self.kit.servo[1]  # Arm extension
                
                # Set servo ranges (adjust for your servos)
                self.motor1.set_pulse_width_range(500, 2500)
                self.motor2.set_pulse_width_range(500, 2500)
                
                self.get_logger().info('Hardware initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Hardware initialization failed: {e}')
                self.kit = None
        else:
            self.kit = None

    def twist_callback(self, msg):
        """Handle Twist messages for Cartesian movement"""
        # Extract desired movement
        dx = msg.linear.x
        dy = msg.linear.y
        
        # Calculate new target position
        target_x = self.current_x + dx
        target_y = self.current_y + dy
        
        self.get_logger().info(f'Moving to: x={target_x:.3f}, y={target_y:.3f}')
        
        # Convert to joint angles using inverse kinematics
        angles = self.inverse_kinematics(target_x, target_y)
        
        if angles:
            self.move_to_angles(angles[0], angles[1])
            self.current_x = target_x
            self.current_y = target_y

    def joint_callback(self, msg):
        """Handle direct joint commands"""
        if len(msg.position) >= 2:
            angle1 = msg.position[0]
            angle2 = msg.position[1]
            
            self.get_logger().info(f'Joint command: j1={math.degrees(angle1):.1f}°, j2={math.degrees(angle2):.1f}°')
            self.move_to_angles(angle1, angle2)

    def inverse_kinematics(self, x, y):
        """Calculate joint angles for desired end-effector position"""
        # 2-DOF planar arm inverse kinematics
        l1, l2 = self.arm_length_1, self.arm_length_2
        
        # Distance to target
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > (l1 + l2) or distance < abs(l1 - l2):
            self.get_logger().warn(f'Target position unreachable: distance={distance:.3f}')
            return None
        
        # Calculate angles
        cos_q2 = (distance*distance - l1*l1 - l2*l2) / (2*l1*l2)
        
        # Ensure cos_q2 is within valid range
        cos_q2 = max(-1, min(1, cos_q2))
        
        q2 = math.acos(cos_q2)  # Elbow up solution
        
        alpha = math.atan2(y, x)
        beta = math.atan2(l2*math.sin(q2), l1 + l2*math.cos(q2))
        q1 = alpha - beta
        
        return [q1, q2]

    def move_to_angles(self, angle1, angle2):
        """Move motors to specified joint angles"""
        self.current_angle1 = angle1
        self.current_angle2 = angle2
        
        if self.kit:
            try:
                # Convert radians to servo degrees (0-180)
                servo1_angle = math.degrees(angle1) + 90  # Offset for servo range
                servo2_angle = math.degrees(angle2) + 90
                
                # Clamp to servo limits
                servo1_angle = max(0, min(180, servo1_angle))
                servo2_angle = max(0, min(180, servo2_angle))
                
                # Move servos
                self.motor1.angle = servo1_angle
                self.motor2.angle = servo2_angle
                
                self.get_logger().info(f'Servo angles: {servo1_angle:.1f}°, {servo2_angle:.1f}°')
                
            except Exception as e:
                self.get_logger().error(f'Motor movement failed: {e}')
        else:
            # Simulation mode
            self.get_logger().info(f'SIMULATION: Moving to angles {math.degrees(angle1):.1f}°, {math.degrees(angle2):.1f}°')

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔌 **Step 5: Hardware Connections**

### For Servo Motors (Recommended for beginners):
```
Raspberry Pi 4B:
├── 5V Power → Servo VCC (Red)
├── GND → Servo GND (Brown/Black)
├── GPIO 18 (Pin 12) → Servo 1 Signal (Orange/Yellow)
└── GPIO 19 (Pin 35) → Servo 2 Signal (Orange/Yellow)

External Power Supply (if needed):
├── 6V Battery/Adapter → Servo VCC
└── GND → Pi GND + Servo GND (common ground)
```

### Install Hardware Dependencies:
```bash
# On Raspberry Pi
sudo apt install -y python3-pip
pip3 install adafruit-circuitpython-servokit
pip3 install RPi.GPIO

# Enable I2C (if using PCA9685 servo driver)
sudo raspi-config
# Navigate to Interface Options → I2C → Enable
```

## 📦 **Step 6: Setup Package**
```bash
# Add entry points to setup.py
cd ~/robot_arm_ws/src/robot_arm_controller
```

Let me create the setup files: 