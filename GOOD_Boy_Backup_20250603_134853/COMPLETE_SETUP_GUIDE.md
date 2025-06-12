# Complete Robot Arm Setup Guide
## Natural Language Control: Cursor → ROS MCP Server → Raspberry Pi → Robot Arm

## 🎯 **System Overview**

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Cursor    │───▶│ ROS MCP     │───▶│ ROS Bridge  │───▶│ Raspberry   │
│ (Ubuntu)    │    │ Server      │    │ WebSocket   │    │ Pi 4B       │
│ Natural     │    │ 172.18.     │    │ Port 9090   │    │ (ROS2)      │
│ Language    │    │ 188.48      │    │             │    │             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                                                                │
                                                                ▼
                                                    ┌─────────────────────┐
                                                    │   2-DOF Robot Arm   │
                                                    │   • 2x Servo Motors │
                                                    │   • 2x Wood Bars    │
                                                    │   • Motor Driver    │
                                                    └─────────────────────┘
```

## 📋 **Part 1: Ubuntu Setup (Your Computer)**

### 1.1 Update MCP Server Configuration
```bash
cd /home/lincen2025/GOOD_Boy/ros-mcp-server

# Update ROSBRIDGE_IP to point to Raspberry Pi
# Edit server.py and change ROSBRIDGE_IP to Pi's IP address
```

### 1.2 Replace server.py with Robot Arm Version
Copy the contents of `robot_arm_server.py` to replace your current `server.py`:

```bash
# Backup current server
cp server.py server_backup.py

# Copy robot arm server (you'll need to do this manually)
# Copy the robot_arm_server.py content to server.py
```

### 1.3 Update Cursor Configuration
The Cursor MCP configuration at `/home/lincen2025/.cursor/mcp.json` should point to the Pi:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/home/lincen2025/GOOD_Boy/ros-mcp-server", 
        "run",
        "server.py"
      ],
      "env": {
        "LOCAL_IP": "172.18.188.48",
        "ROSBRIDGE_IP": "PI_IP_ADDRESS",
        "ROSBRIDGE_PORT": "9090"
      }
    }
  }
}
```

## 📋 **Part 2: Raspberry Pi Setup**

### 2.1 Install Raspberry Pi OS (64-bit)
```bash
# Flash Raspberry Pi OS to SD card
# Enable SSH and WiFi during setup
# Boot and connect via SSH

ssh pi@PI_IP_ADDRESS
```

### 2.2 Install ROS2 Humble
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-humble-ros-base python3-argcomplete
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Install ROS Bridge
```bash
sudo apt install -y ros-humble-rosbridge-server
```

### 2.4 Install Hardware Dependencies
```bash
# Python packages for servo control
sudo apt install -y python3-pip
pip3 install adafruit-circuitpython-servokit
pip3 install RPi.GPIO

# Enable I2C
sudo raspi-config
# Navigate to Interface Options → I2C → Enable → Reboot
```

### 2.5 Create Robot Arm Package
```bash
# Create workspace
mkdir -p ~/robot_arm_ws/src
cd ~/robot_arm_ws/src

# Create package
ros2 pkg create --build-type ament_python robot_arm_controller \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs

# Navigate to package directory
cd robot_arm_controller/robot_arm_controller

# Create Python files (copy content from the setup files)
# You'll need to manually create these files:
# - motor_controller.py
# - joint_state_publisher.py  
# - cartesian_controller.py

# Create launch directory
mkdir -p ../../launch

# Copy launch file
# Copy robot_arm_launch.py to ~/robot_arm_ws/src/robot_arm_controller/launch/

# Update setup.py with the setup.py content provided

# Build the package
cd ~/robot_arm_ws
colcon build
source install/setup.bash
```

## 📋 **Part 3: Hardware Setup**

### 3.1 Servo Motor Connections
```
Raspberry Pi 4B GPIO:
├── Pin 2 (5V) → Servo VCC (Red wires)
├── Pin 6 (GND) → Servo GND (Black/Brown wires)  
├── Pin 12 (GPIO 18) → Servo 1 Signal (Yellow/Orange)
└── Pin 35 (GPIO 19) → Servo 2 Signal (Yellow/Orange)

For PCA9685 Servo Driver (recommended):
├── Pin 3 (SDA) → PCA9685 SDA
├── Pin 5 (SCL) → PCA9685 SCL
├── Pin 2 (5V) → PCA9685 VCC
├── Pin 6 (GND) → PCA9685 GND
└── Servos connect to PCA9685 channels 0 and 1
```

### 3.2 Robot Arm Assembly
```
Base Assembly:
├── Servo 1 (Base) → Fixed to base plate
├── Servo 1 Horn → Attached to Wood Bar 1 (15cm)
├── Wood Bar 1 → Connected to Servo 2
├── Servo 2 Horn → Attached to Wood Bar 2 (10cm)
└── End Effector (optional) → Attached to Wood Bar 2 end

Coordinate System:
├── X-axis: Left(-) to Right(+)
├── Y-axis: Down(-) to Up(+)  
└── Origin: Base servo center
```

## 📋 **Part 4: Network Configuration**

### 4.1 Find Raspberry Pi IP Address
```bash
# On Raspberry Pi
hostname -I | awk '{print $1}'
# Note this IP address (e.g., 192.168.1.100)
```

### 4.2 Update Ubuntu MCP Server
```bash
# On Ubuntu computer
cd /home/lincen2025/GOOD_Boy/ros-mcp-server

# Edit server.py or robot_arm_server.py
# Update ROSBRIDGE_IP to Pi's IP address
ROSBRIDGE_IP = "192.168.1.100"  # Use your Pi's actual IP
```

### 4.3 Update Cursor Configuration
```bash
# Edit /home/lincen2025/.cursor/mcp.json
# Update ROSBRIDGE_IP environment variable
```

## 📋 **Part 5: Testing and Operation**

### 5.1 Start Raspberry Pi System
```bash
# SSH into Raspberry Pi
ssh pi@PI_IP_ADDRESS

# Navigate to workspace
cd ~/robot_arm_ws
source install/setup.bash

# Launch robot arm system
ros2 launch robot_arm_controller robot_arm_launch.py

# Or start components individually:
# Terminal 1: ros2 run robot_arm_controller joint_state_publisher
# Terminal 2: ros2 run robot_arm_controller motor_controller  
# Terminal 3: ros2 run robot_arm_controller cartesian_controller
# Terminal 4: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 5.2 Test ROS Bridge Connection
```bash
# On Ubuntu, test connection to Pi
curl -v ws://PI_IP_ADDRESS:9090

# Should connect successfully to WebSocket
```

### 5.3 Start Ubuntu MCP Server
```bash
# On Ubuntu
cd /home/lincen2025/GOOD_Boy/ros-mcp-server
source .venv/bin/activate
python server.py

# Should connect to Pi's ROS Bridge
```

### 5.4 Test with Cursor
Open Cursor and try these natural language commands:

**Basic Movement Commands:**
- "Move 2 cm along x direction"
- "Move the arm 5 cm to the right"
- "Move 3 cm up"
- "Go to position (10, 15)"

**Joint Commands:**
- "Move joint 1 to 45 degrees"
- "Set joints to 30 and 60 degrees"

**Control Commands:**
- "Move arm to home position"
- "Stop the arm"
- "Get available topics"

## 🎯 **Expected Natural Language Commands**

| Your Command | What Happens |
|--------------|--------------|
| "Move 2 cm along x direction" | Arm moves 2cm right |
| "Move 5 cm to the left" | Arm moves 5cm left |
| "Move up 3 centimeters" | Arm moves 3cm up |
| "Go to position (10, 15)" | Arm moves to (10,15)cm |
| "Move joint 1 to 45 degrees" | Base servo rotates to 45° |
| "Home the arm" | Arm returns to center position |
| "Stop arm movement" | All movement stops immediately |

## 🐛 **Troubleshooting**

### Connection Issues:
```bash
# Check Pi ROS Bridge status
ros2 topic list
ros2 topic echo /joint_states

# Check network connectivity  
ping PI_IP_ADDRESS
telnet PI_IP_ADDRESS 9090
```

### Hardware Issues:
```bash
# Test servo movement on Pi
python3 -c "
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
kit.servo[0].angle = 90
kit.servo[1].angle = 90
"
```

### ROS2 Issues:
```bash
# Check ROS2 nodes
ros2 node list
ros2 topic list
ros2 service list

# Monitor joint states
ros2 topic echo /joint_states
```

## 🎉 **Success Indicators**

✅ **Pi ROS2 nodes running**  
✅ **ROS Bridge responding on port 9090**  
✅ **Ubuntu MCP server connecting to Pi**  
✅ **Cursor recognizing MCP functions**  
✅ **Natural language commands moving arm**  
✅ **Servo motors responding correctly**  

Your robot arm is now ready for natural language control! 🤖 