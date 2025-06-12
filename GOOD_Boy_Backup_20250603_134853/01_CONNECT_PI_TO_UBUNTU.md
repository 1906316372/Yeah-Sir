# Step 1: Connect Raspberry Pi to Ubuntu

## 🎯 **Goal**: Establish network communication between Ubuntu PC and Raspberry Pi

## 📋 **Prerequisites**
- Raspberry Pi 4B with fresh Raspberry Pi OS installed
- Ubuntu machine (your computer)
- Both devices on the same network (WiFi or Ethernet)

## 🔧 **Step 1.1: Enable SSH on Raspberry Pi**

### Option A: Enable SSH during Pi OS installation (Recommended)
1. **Flash Raspberry Pi OS** to SD card using Raspberry Pi Imager
2. **Before ejecting**, click the gear icon ⚙️ for advanced options
3. **Check "Enable SSH"** and set a password
4. **Configure WiFi** with your network credentials
5. **Set username** (e.g., `pi`) and **password**
6. Flash and boot the Pi

### Option B: Enable SSH after booting (if you missed it)
1. Connect keyboard/mouse/monitor to Pi
2. Open terminal and run:
```bash
sudo systemctl enable ssh
sudo systemctl start ssh
```

## 🔧 **Step 1.2: Find Raspberry Pi IP Address**

### Method 1: From Ubuntu - Network Scan
```bash
# On Ubuntu, scan your local network
# First, find your network range
ip route | grep 'scope link'

# Scan for devices (adjust range based on your network)
nmap -sn 192.168.1.0/24
# or
nmap -sn 192.168.0.0/24

# Look for "Raspberry Pi Foundation" in the results
```

### Method 2: From Router Admin Panel
1. Open your router's web interface (usually `192.168.1.1` or `192.168.0.1`)
2. Look for connected devices
3. Find "Raspberry Pi" or device with MAC starting with `B8:27:EB`, `DC:A6:32`, or `E4:5F:01`

### Method 3: Directly on Pi (if you have monitor)
```bash
# On Raspberry Pi terminal
hostname -I
ifconfig
```

## 🔧 **Step 1.3: Test SSH Connection**

Once you have the Pi's IP address, test the connection:

```bash
# On Ubuntu - Replace with your Pi's actual IP
ssh pi@192.168.1.XXX

# Example:
ssh pi@192.168.1.100

# First time connection will ask to trust the certificate - type 'yes'
# Enter the password you set during installation
```

**✅ Success indicator**: You should see the Raspberry Pi command prompt:
```
pi@raspberrypi:~ $
```

## 🔧 **Step 1.4: Configure Static IP (Optional but Recommended)**

To prevent the Pi's IP from changing, set up a static IP:

### On Raspberry Pi:
```bash
# SSH into Pi first
ssh pi@PI_IP_ADDRESS

# Edit dhcpcd configuration
sudo nano /etc/dhcpcd.conf

# Add these lines at the end (adjust for your network):
interface wlan0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8

# Save (Ctrl+O) and exit (Ctrl+X)

# Restart networking
sudo systemctl restart dhcpcd

# Reboot to apply changes
sudo reboot
```

## 🔧 **Step 1.5: Update Pi and Install Basic Tools**

```bash
# SSH back into Pi after reboot
ssh pi@192.168.1.100  # Use your Pi's IP

# Update system
sudo apt update && sudo apt upgrade -y

# Install useful tools
sudo apt install -y curl wget git nano htop

# Reboot if kernel was updated
sudo reboot
```

## 🔧 **Step 1.6: Verify Network Configuration**

### On Ubuntu, test the connection:
```bash
# Ping test
ping 192.168.1.100  # Replace with your Pi's IP

# SSH test
ssh pi@192.168.1.100

# Check if Pi can reach Ubuntu
# On Pi:
ping 172.18.188.48  # Your Ubuntu IP
```

## 🔧 **Step 1.7: Set Up Key-Based SSH (Optional)**

For easier access without passwords:

```bash
# On Ubuntu - Generate SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096

# Copy key to Pi
ssh-copy-id pi@192.168.1.100

# Test passwordless login
ssh pi@192.168.1.100
```

## 🔧 **Step 1.8: Create Network Configuration File**

Create a reference file with your network setup:

```bash
# On Ubuntu
cd /home/lincen2025/GOOD_Boy/ros-mcp-server

# Create network config file
cat > network_config.txt << EOF
# Network Configuration for Robot Arm Project
# Generated: $(date)

Ubuntu Machine:
- IP Address: 172.18.188.48
- Hostname: $(hostname)

Raspberry Pi:
- IP Address: 192.168.1.100  # UPDATE WITH ACTUAL IP
- Username: pi
- SSH Command: ssh pi@192.168.1.100

Network Test Commands:
# From Ubuntu to Pi:
ping 192.168.1.100
ssh pi@192.168.1.100

# From Pi to Ubuntu:
ping 172.18.188.48

ROS Bridge Configuration:
- Pi will run ROS Bridge on port 9090
- Ubuntu MCP Server will connect to: ws://192.168.1.100:9090
EOF

# View the file
cat network_config.txt
```

## ✅ **Verification Checklist**

Test these before proceeding to the next step:

- [ ] **Pi boots and connects to WiFi**
- [ ] **Can SSH from Ubuntu to Pi**: `ssh pi@PI_IP`  
- [ ] **Pi can ping Ubuntu**: `ping 172.18.188.48`
- [ ] **Ubuntu can ping Pi**: `ping PI_IP`
- [ ] **Static IP configured** (if desired)
- [ ] **Pi system updated**

## 🎯 **Next Steps**

Once connection is established:

1. **Update network_config.txt** with your actual Pi IP address
2. **Update MCP server configuration** with Pi IP
3. **Proceed to installing ROS2 on the Pi**

## 🐛 **Common Issues and Solutions**

### Pi Not Found on Network
```bash
# Check if Pi is connected to correct WiFi
# On Pi (with monitor):
iwconfig
sudo raspi-config  # Network Options → WiFi
```

### SSH Connection Refused
```bash
# Enable SSH on Pi
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH status
sudo systemctl status ssh
```

### Can't Remember Pi IP
```bash
# Find Pi using hostname
ping raspberrypi.local

# Or use router's DHCP table
arp -a | grep -i "b8:27:eb\|dc:a6:32\|e4:5f:01"
```

### Network Permission Issues
```bash
# If ping/SSH requires sudo
sudo ufw allow ssh
sudo ufw allow from 172.18.188.48  # Allow Ubuntu IP
```

---

**🎉 Once this step is complete, you'll have:**
- ✅ Stable network connection between Ubuntu and Pi
- ✅ SSH access to Pi from Ubuntu  
- ✅ Network configuration documented
- ✅ Ready for ROS2 installation

**📝 Record your Pi's IP address**: `___________________`

This IP will be used in all subsequent configuration steps! 