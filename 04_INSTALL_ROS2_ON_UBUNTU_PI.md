# Step 4: Install ROS2 Humble on Ubuntu Server Pi

## 🎯 **Goal**: Install ROS2 Humble on Ubuntu Server 22.04.5 LTS (Native Support!)

## 📋 **Current Setup** ✅
- **Pi IP**: `RASPBERRY_PI_IP`
- **Ubuntu IP**: `UBUNTU_MACHINE_IP` 
- **Pi OS**: Ubuntu Server 22.04.5 LTS (aarch64)
- **Python**: 3.10.x (perfect for ROS2 Humble!)
- **SSH**: `ssh PI_USERNAME@RASPBERRY_PI_IP`

## 🚀 **Why This Will Work Perfectly**
- ✅ **Native Ubuntu**: No compatibility issues
- ✅ **Python 3.10**: Exact match for ROS2 Humble
- ✅ **ARM64**: Optimized packages available
- ✅ **apt install**: Simple package installation

## 🔧 **Step 4.1: Update System & Add ROS2 Repository**

Run these commands on your **Pi** (via SSH):

```bash
# SSH into Pi
ssh PI_USERNAME@RASPBERRY_PI_IP

# Update system packages
sudo apt update && sudo apt upgrade -y

# Set locale for ROS2
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install essential tools
sudo apt install -y software-properties-common curl

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update
```

## 🔧 **Step 4.2: Install ROS2 Humble**

```bash
# Install ROS2 Humble (this will work perfectly now!)
sudo apt install -y ros-humble-ros-base

# Install development tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-argcomplete

# Install essential ROS2 packages
sudo apt install -y ros-humble-geometry-msgs
sudo apt install -y ros-humble-sensor-msgs  
sudo apt install -y ros-humble-std-msgs

# Initialize rosdep
sudo rosdep init
rosdep update
```

## 🔧 **Step 4.3: Install ROS Bridge Server**

```bash
# Install ROS Bridge for MCP communication
sudo apt install -y ros-humble-rosbridge-server
sudo apt install -y ros-humble-rosbridge-library

# Verify installation
dpkg -l | grep ros-humble-rosbridge
```

## 🔧 **Step 4.4: Setup ROS2 Environment**

```bash
# Add ROS2 to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Source for current session
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Verify ROS2 installation
ros2 --help
```

## 🔧 **Step 4.5: Test ROS2 Installation**

```bash
# Test 1: Check ROS2 commands
ros2 topic list
ros2 node list

# Test 2: Run demo (open second SSH session)
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2 (new SSH session):
ssh PI_USERNAME@RASPBERRY_PI_IP
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener

# You should see: "Hello World: X" messages
# Press Ctrl+C to stop both
```

## 🔧 **Step 4.6: Test ROS Bridge**

```bash
# Start ROS Bridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Should show:
# [INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Test from Ubuntu machine** (new terminal):

```bash
# On Ubuntu machine, test WebSocket connection
curl -v ws://RASPBERRY_PI_IP:9090

# Should connect successfully (Ctrl+C to exit)
```

## 🔧 **Step 4.7: Install Hardware Dependencies**

Install Python libraries for servo control:

```bash
# On Pi
sudo apt install -y python3-pip

# Install servo control libraries
pip3 install adafruit-circuitpython-servokit
pip3 install RPi.GPIO

# Enable I2C for PCA9685 servo driver
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable → Finish

# Verify I2C
lsmod | grep i2c
```

## ✅ **Verification Checklist**

Verify everything works:

- [ ] **ROS2 commands**: `ros2 topic list` works
- [ ] **Demo nodes**: talker/listener communicate
- [ ] **ROS Bridge**: Starts on port 9090
- [ ] **Ubuntu connection**: `curl -v ws://RASPBERRY_PI_IP:9090` connects
- [ ] **Python libraries**: `python3 -c "import RPi.GPIO"` works
- [ ] **I2C enabled**: `lsmod | grep i2c` shows modules

## 🎯 **Next Steps**

Once ROS2 is working:

1. ✅ **Create robot arm ROS2 package** on Pi
2. ✅ **Test MCP server connection** from Ubuntu
3. ✅ **Set up hardware connections** (servo motors)
4. ✅ **Test end-to-end communication**: Cursor → Ubuntu → Pi → Motors

## 🐛 **Common Issues & Solutions**

### Import Error
```bash
# If rosdep fails
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

### ROS Bridge Port Issues
```bash
# Check if port 9090 is free
sudo netstat -tulpn | grep 9090

# Kill existing processes
sudo pkill -f rosbridge
```

### Permission Issues
```bash
# Add user to dialout group for hardware access
sudo usermod -a -G dialout $USER
```

## 📝 **Current Status**

- ✅ **Network**: Ubuntu ↔ Pi (RASPBERRY_PI_IP) established
- ✅ **OS**: Ubuntu Server 22.04.5 LTS running perfectly  
- 🔄 **ROS2**: Installing Humble (current step)
- ⏳ **Robot Package**: After ROS2 complete
- ⏳ **Hardware**: After software setup
- ⏳ **Integration**: Final step

**Ready to install ROS2?** Run the commands in Step 4.1 on your Pi! 🚀 