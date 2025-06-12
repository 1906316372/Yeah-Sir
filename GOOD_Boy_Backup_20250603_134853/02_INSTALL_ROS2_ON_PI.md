# Step 2: Install ROS2 Humble on Raspberry Pi OS

## 🎯 **Goal**: Install ROS2 Humble on your Raspberry Pi running Pi OS

## 📋 **Current Setup**
- **Pi IP**: `192.168.131.5`
- **Ubuntu IP**: `192.168.131.32` 
- **Pi OS**: Raspberry Pi OS (Debian-based)
- **Connection**: ✅ SSH working

## 🔧 **Step 2.1: Update Raspberry Pi System**

First, let's ensure your Pi is fully updated:

```bash
# SSH into your Pi (from Ubuntu)
ssh LCen@192.168.131.5

# Update package lists and system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y curl gnupg2 lsb-release build-essential

# Reboot if kernel was updated
sudo reboot
```

## 🔧 **Step 2.2: Set Locale for ROS2**

```bash
# SSH back in after reboot
ssh LCen@192.168.131.5

# Set up locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale
locale
```

## 🔧 **Step 2.3: Add ROS2 Repository**

```bash
# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo apt update

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update
```

## 🔧 **Step 2.4: Install ROS2 Humble**

```bash
# Install ROS2 base (lighter for Pi)
sudo apt install -y ros-humble-ros-base

# Install Python3 colcon and other tools
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-argcomplete

# Install additional ROS2 packages we'll need
sudo apt install -y ros-humble-geometry-msgs
sudo apt install -y ros-humble-sensor-msgs
sudo apt install -y ros-humble-std-msgs

# Initialize rosdep
sudo rosdep init
rosdep update
```

## 🔧 **Step 2.5: Install ROS Bridge Server**

```bash
# Install rosbridge for communication with MCP server
sudo apt install -y ros-humble-rosbridge-server
sudo apt install -y ros-humble-rosbridge-library

# Verify installation
dpkg -l | grep ros-humble-rosbridge
```

## 🔧 **Step 2.6: Setup ROS2 Environment**

```bash
# Add ROS2 setup to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Source the environment for current session
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Verify ROS2 installation
ros2 --help
```

## 🔧 **Step 2.7: Test ROS2 Installation**

Let's test that ROS2 is working properly:

```bash
# Test 1: Check ROS2 commands
ros2 topic list
ros2 node list

# Test 2: Run demo nodes (in separate terminals)
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2 (new SSH session):
ssh LCen@192.168.131.5
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener

# You should see messages being passed between nodes
# Press Ctrl+C to stop both
```

## 🔧 **Step 2.8: Test ROS Bridge**

```bash
# Start ROS Bridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# You should see output like:
# [INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Keep this running** and **test from Ubuntu** in a new terminal:

```bash
# On Ubuntu machine, test connection to Pi's ROS Bridge
curl -v ws://192.168.131.5:9090

# Should connect successfully (Ctrl+C to exit)
```

## 🔧 **Step 2.9: Install Hardware Dependencies**

For controlling servo motors, install Python libraries:

```bash
# On Raspberry Pi
sudo apt install -y python3-pip

# Install servo control libraries
pip3 install adafruit-circuitpython-servokit
pip3 install RPi.GPIO

# Enable I2C for servo driver (PCA9685)
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable → Finish

# Verify I2C is enabled
lsmod | grep i2c
```

## 🔧 **Step 2.10: Create Network Configuration File**

Let's update our network configuration:

```bash
# On Ubuntu
cd /home/lincen2025/GOOD_Boy/ros-mcp-server

# Update network config with actual IPs
cat > network_config.txt << EOF
# Network Configuration for Robot Arm Project
# Updated: $(date)

Ubuntu Machine:
- IP Address: 192.168.131.32
- Hostname: $(hostname)

Raspberry Pi:
- IP Address: 192.168.131.5
- Hostname: GBraspberrypi
- Username: LCen
- SSH Command: ssh LCen@192.168.131.5

Network Test Commands:
# From Ubuntu to Pi:
ping 192.168.131.5
ssh LCen@192.168.131.5

# From Pi to Ubuntu:
ping 192.168.131.32

ROS Bridge Configuration:
- Pi runs ROS Bridge on: ws://192.168.131.5:9090
- Ubuntu MCP Server connects to: ws://192.168.131.5:9090

ROS2 Status:
- ✅ ROS2 Humble installed on Pi
- ✅ ROS Bridge server installed
- ✅ Hardware libraries installed
EOF

# View the configuration
cat network_config.txt
```

## ✅ **Verification Checklist**

Before proceeding to the next step, verify:

- [ ] **ROS2 commands work**: `ros2 topic list`
- [ ] **Demo nodes communicate**: talker/listener test passes
- [ ] **ROS Bridge starts**: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- [ ] **Ubuntu can connect**: `curl -v ws://192.168.131.5:9090`
- [ ] **I2C enabled**: `lsmod | grep i2c`
- [ ] **Python libraries installed**: `python3 -c "import RPi.GPIO"`

## 🐛 **Common Issues & Solutions**

### ROS2 Repository Issues
```bash
# If repository key fails
wget -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
```

### ROS Bridge Port Issues
```bash
# Check if port 9090 is available
sudo netstat -tulpn | grep 9090

# Kill any process using port 9090
sudo pkill -f rosbridge
```

### I2C Not Working
```bash
# Enable I2C manually
echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/config.txt
sudo reboot
```

## 🎯 **Next Steps**

Once ROS2 is working:

1. ✅ **Update MCP server** to connect to `192.168.131.5:9090`
2. ✅ **Create robot arm ROS2 package** on the Pi
3. ✅ **Set up hardware connections** for servo motors
4. ✅ **Test end-to-end communication** Ubuntu → Pi → Motors

## 📝 **Current Status**

- ✅ **Network**: Ubuntu ↔ Pi connection established
- 🔄 **ROS2**: Installing on Pi (current step)
- ⏳ **Hardware**: Next step
- ⏳ **Robot Package**: After hardware
- ⏳ **Integration**: Final step

**Ready to proceed?** Let me know when Step 2 is complete and we'll move to creating the robot arm package! 