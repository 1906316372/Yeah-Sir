# GOOD_Boy 2-DOF Robot Arm - Complete Backup Package

**Backup Date**: June 3, 2025  
**System Status**: Fully functional with 360° unlimited rotation capability

## 🚀 Quick Start Restoration Guide

This backup contains everything needed to restore your GOOD_Boy robot arm project on any new computer.

### 📋 Hardware Requirements
- **Raspberry Pi 4B** (Ubuntu Server 22.04.5 LTS)
- **2x NEMA 11 Stepper Motors** with 13.73:1 planetary gearboxes
- **2x A4988 Stepper Drivers**
- **Wooden arm segments**: 15cm base + 10cm end effector
- **Network**: Ubuntu machine ↔ Raspberry Pi (WiFi/Ethernet)

### 🔧 System Architecture
```
Cursor IDE → MCP Server → ROS Bridge WebSocket → ROS2 Humble → GPIO + Stepper Control
```

## 📁 Backup Contents

### Core Project Files
- `working_mcp_functions.py` - **WORKING** MCP-style robot control functions
- `*_unlocked.py` - Unlimited rotation versions (servo_controller, arm_kinematics)
- `*_limited.py` - Original limited versions (backup)
- Movement test scripts: `simple_move_2cm_x.py`, `spin_720_degrees.py`, etc.

### ROS2 Workspace
- `pi_ros2_workspace/` - **Complete ROS2 workspace from Raspberry Pi**
  - Source code with unlimited rotation modifications
  - Built packages ready for installation
  - Launch files and configuration

### Documentation
- `01-05_*.md` - Step-by-step setup guides
- `COMPLETE_SETUP_GUIDE.md` - Original setup documentation
- Hardware connection diagrams and configuration files

### Network & MCP Configuration
- `cursor_mcp_config.json` - MCP server configuration
- `network_config.txt` - Network setup details
- SSH key setup instructions

## 🛠️ Restoration Instructions

### 1. Raspberry Pi Setup

**Flash Ubuntu Server 22.04.5 LTS** to Pi following `03_FLASH_UBUNTU_SERVER_TO_PI.md`

**Connect to network** using `01_CONNECT_PI_TO_UBUNTU.md`

**Install ROS2 Humble**:
```bash
# Follow 04_INSTALL_ROS2_ON_UBUNTU_PI.md or run:
sudo apt update && sudo apt upgrade -y
wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install python3-rosdep2 python3-colcon-common-extensions -y
```

**Restore ROS2 workspace**:
```bash
# Copy the workspace to Pi
scp -r pi_ros2_workspace/* LCen@PI_IP:~/ros2_ws/

# On Pi, rebuild packages
cd ~/ros2_ws
colcon build --packages-select robot_arm_controller
source install/setup.bash
```

**Install rosbridge**:
```bash
sudo apt install ros-humble-rosbridge-suite -y
```

### 2. Ubuntu Machine Setup

**Install dependencies**:
```bash
sudo apt install python3-pip python3-websocket -y
pip3 install websocket-client
```

**Setup SSH keys** (passwordless access):
```bash
ssh-keygen -t rsa -b 4096
ssh-copy-id LCen@PI_IP
```

**Configure MCP** (if using Cursor IDE):
```bash
cp cursor_mcp_config.json ~/.cursor/mcp_config.json
```

### 3. System Startup

**On Raspberry Pi**:
```bash
# Terminal 1: Start ROS Bridge
cd ~/ros2_ws && source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start Robot Arm Controller
cd ~/ros2_ws && source install/setup.bash
ros2 launch robot_arm_controller robot_arm_launch.py
```

**On Ubuntu Machine**:
```bash
# Test connection
python3 working_mcp_functions.py

# Use any movement script
python3 simple_move_2cm_x.py
```

## 🎯 Key Features Restored

### ✅ **360° Unlimited Rotation**
- No angle limits (-999° to +999°)
- No workspace distance restrictions
- Full inverse kinematics for any position

### ✅ **Working MCP Functions**
```python
# Direct WebSocket control (bypasses broken MCP server)
from working_mcp_functions import *

get_topics()           # List ROS topics
pub_twist([0,0,0], [0,0,0])  # Send velocity commands
pub_jointstate(["joint1","joint2"], [90,45], [0,0], [0,0])  # Joint control
stop_all_motion()      # Emergency stop
```

### ✅ **High Precision Control**
- **43,936 steps/revolution** = **0.0082° per step**
- 1/16 microstepping with planetary gearboxes
- Real-time position feedback

### ✅ **Emergency Stop Procedures**
```bash
# Soft stop (preferred)
python3 -c "from working_mcp_functions import stop_all_motion; stop_all_motion()"

# Hard stop (if needed)
ssh PI_USERNAME@PI_IP "pkill -f robot_arm"
```

## 🔗 Network Configuration

**Default IPs** (update as needed):
- Ubuntu Machine: `UBUNTU_MACHINE_IP`
- Raspberry Pi: `RASPBERRY_PI_IP`
- ROS Bridge: `ws://RASPBERRY_PI_IP:9090`

Update IP addresses in:
- `working_mcp_functions.py` (line 8: `ROSBRIDGE_URL`)
- Movement scripts (`simple_move_2cm_x.py`, etc.)

## 🧪 Testing the Restoration

**1. Basic connectivity**:
```bash
python3 working_mcp_functions.py
# Should show: Topics=✅, Stop=✅
```

**2. Small movement test**:
```bash
python3 simple_move_2cm_x.py
# Should move arm 2cm in X direction
```

**3. Advanced test**:
```bash
python3 test_360_rotation.py
# Tests unlimited rotation capability
```

## 🚨 Important Safety Notes

- **Always verify emergency stop** before major movements
- **Motors have NO physical limits** - software controls everything
- **Start with small movements** to verify calibration
- **Keep emergency stop readily available**: `working_mcp_functions.stop_all_motion()`

## 📞 Troubleshooting

**Connection Issues**:
- Verify network: `ping PI_IP`
- Check ROS Bridge: `wget -qO- http://PI_IP:9090` (should show WebSocket info)
- Test SSH: `ssh PI_USERNAME@PI_IP`

**Robot Not Moving**:
- Check processes: `ssh PI_USERNAME@PI_IP "ps aux | grep robot_arm"`
- Restart services: Follow startup instructions
- Verify GPIO permissions: `sudo usermod -a -G gpio PI_USERNAME`

**MCP Server Issues**:
- **Use `working_mcp_functions.py`** instead of broken MCP server tools
- Original MCP tools have WebSocket library compatibility issues

## 🎉 Success Indicators

When properly restored, you should see:
- WebSocket connection to `ws://PI_IP:9090`
- ROS topics available via `get_topics()`
- Smooth motor movement with position logging
- Emergency stop working reliably

---

**Your GOOD_Boy robot arm system is fully backed up and ready for restoration!**

For detailed step-by-step instructions, refer to the numbered guide files (`01_*` through `05_*`). 