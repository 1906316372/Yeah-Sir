# Step 3: Flash Ubuntu Server 22.04 LTS to Raspberry Pi

## 🎯 **Goal**: Replace Raspberry Pi OS with Ubuntu Server 22.04 LTS for native ROS2 support

## 📋 **What You'll Need**
- **SD Card**: Your current Pi SD card (will be wiped)
- **Card Reader**: To connect SD card to your Ubuntu machine
- **Backup**: Save any important data from current Pi first!

## 🔧 **Step 3.1: Download Ubuntu Server Image**

On your **Ubuntu machine**:

```bash
# Create downloads directory
cd ~/Downloads

# Download Ubuntu Server 22.04.4 LTS (ARM64) for Raspberry Pi
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.4-preinstalled-server-arm64+raspi.img.xz

# Verify download size (should be ~1GB compressed)
ls -lh ubuntu-22.04.4-preinstalled-server-arm64+raspi.img.xz
```

## 🔧 **Step 3.2: Install Raspberry Pi Imager**

```bash
# Install Raspberry Pi Imager (easiest method)
sudo snap install rpi-imager

# Alternative: Download from website
# wget https://downloads.rpilocator.com/imager/rpi-imager-1.8.5-amd64.deb
# sudo dpkg -i rpi-imager-1.8.5-amd64.deb
```

## 🔧 **Step 3.3: Flash the SD Card**

### **Method 1: Using Raspberry Pi Imager (Recommended)**

1. **Insert SD card** into your Ubuntu machine
2. **Launch Imager**:
   ```bash
   rpi-imager
   ```
3. **Configure settings**:
   - Click "CHOOSE OS" → "Use custom" → Select the downloaded `.img.xz` file
   - Click "CHOOSE STORAGE" → Select your SD card
   - Click **⚙️ Settings** (gear icon) and configure:
     - ✅ **Enable SSH**: Use password authentication
     - **Username**: `PI_USERNAME` (same as before)
     - **Password**: `your_password`
     - **Hostname**: `ubuntu-pi` 
     - ✅ **Configure WiFi**: Enter your network details
     - **WiFi Country**: Your country code
4. **Flash**: Click "WRITE" and wait (~10-15 minutes)

### **Method 2: Command Line (Alternative)**

```bash
# Find your SD card device
lsblk

# Extract and flash (replace /dev/sdX with your SD card)
xz -d ubuntu-22.04.4-preinstalled-server-arm64+raspi.img.xz
sudo dd if=ubuntu-22.04.4-preinstalled-server-arm64+raspi.img of=/dev/sdX bs=4M status=progress
sudo sync
```

## 🔧 **Step 3.4: Configure Network & SSH (If not done in Imager)**

If you used command line or need to configure manually:

```bash
# Mount the SD card boot partition
sudo mkdir -p /media/boot
sudo mount /dev/sdX1 /media/boot  # Replace X with your SD card letter

# Enable SSH
sudo touch /media/boot/ssh

# Configure WiFi (if using WiFi)
cat << EOF | sudo tee /media/boot/wpa_supplicant.conf
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
network={
    ssid=WIFI_SSID_PLACEHOLDER
    psk=WIFI_PASSWORD_PLACEHOLDER
}
EOF

# Unmount
sudo umount /media/boot
```

## 🔧 **Step 3.5: First Boot Setup**

1. **Insert SD card** into Raspberry Pi
2. **Power on** the Pi
3. **Wait 3-5 minutes** for first boot (Ubuntu expands filesystem)
4. **Find Pi's new IP**:
   ```bash
   # From your Ubuntu machine
   nmap -sn 192.168.131.0/24 | grep -B2 -A2 "ubuntu"
   
   # Or check your router's device list
   # The hostname should be "ubuntu-pi"
   ```

## 🔧 **Step 3.6: SSH into Ubuntu Pi**

```bash
# SSH with new credentials (IP might be different now)
ssh PI_USERNAME@NEW_PI_IP

# First login will ask to change password
# Follow the prompts to set a new password
```

## 🔧 **Step 3.7: Initial Ubuntu Setup**

Once connected to your Ubuntu Pi:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y curl wget git nano htop

# Check Ubuntu version
lsb_release -a
# Should show: Ubuntu 22.04.4 LTS

# Check Python version
python3 --version
# Should show: Python 3.10.x

# Reboot to ensure all updates applied
sudo reboot
```

## 🔧 **Step 3.8: Verify Network Configuration**

```bash
# SSH back in after reboot
ssh PI_USERNAME@NEW_PI_IP

# Check network
ip addr show
hostname -I

# Test connectivity
ping google.com
ping UBUNTU_MACHINE_IP  # Your Ubuntu machine
```

## 📝 **Update Network Configuration**

Once you have the new IP, update our config:

```bash
# On your Ubuntu machine
cd /home/USERNAME/GOOD_Boy/ros-mcp-server

# Update network config with new IP
cat > network_config.txt << EOF
# Network Configuration for Robot Arm Project
# Updated: $(date)

Ubuntu Machine:
- IP Address: UBUNTU_MACHINE_IP
- Hostname: $(hostname)

Raspberry Pi (Ubuntu Server):
- IP Address: NEW_PI_IP_HERE
- Hostname: ubuntu-pi
- Username: PI_USERNAME
- SSH Command: ssh PI_USERNAME@NEW_PI_IP_HERE
- OS: Ubuntu Server 22.04 LTS ARM64

Network Test Commands:
# From Ubuntu to Pi:
ping NEW_PI_IP_HERE
ssh PI_USERNAME@NEW_PI_IP_HERE

# From Pi to Ubuntu:
ping UBUNTU_MACHINE_IP

ROS Bridge Configuration:
- Pi runs ROS Bridge on: ws://NEW_PI_IP_HERE:9090
- Ubuntu MCP Server connects to: ws://NEW_PI_IP_HERE:9090

ROS2 Status:
- ⏳ Ubuntu Server 22.04 installed
- ⏳ ROS2 Humble ready to install
- ⏳ Hardware libraries ready to install
EOF
```

## ✅ **Verification Checklist**

Before proceeding to ROS2 installation:

- [ ] **Pi boots**: Ubuntu Server starts successfully
- [ ] **SSH works**: Can connect via `ssh PI_USERNAME@NEW_PI_IP`
- [ ] **Network**: Pi can ping Ubuntu machine and internet
- [ ] **Python 3.10**: `python3 --version` shows 3.10.x
- [ ] **Ubuntu 22.04**: `lsb_release -a` shows 22.04 LTS

## 🎯 **Next Steps**

Once Ubuntu Server is running:

1. ✅ **Install ROS2 Humble** (will work perfectly now!)
2. ✅ **Install ROS Bridge**
3. ✅ **Update MCP server** with new Pi IP
4. ✅ **Create robot arm package**
5. ✅ **Test end-to-end communication**

## 🐛 **Common Issues & Solutions**

### Pi Won't Boot
- Wait 5+ minutes for first boot
- Check SD card connection
- Try re-flashing if LED patterns indicate boot failure

### Can't Find Pi IP
```bash
# Scan network more thoroughly
sudo nmap -sn 192.168.131.0/24
sudo nmap -sn 192.168.1.0/24  # Try different subnet

# Connect via ethernet cable temporarily
# Or check router admin panel
```

### SSH Connection Refused
```bash
# Wait longer - SSH starts after boot completes
# Or connect monitor/keyboard to Pi for direct access
```

## 📝 **Current Status**

- ✅ **Network**: Ubuntu ↔ Pi connection established  
- 🔄 **OS**: Flashing Ubuntu Server (current step)
- ⏳ **ROS2**: Install after Ubuntu is ready
- ⏳ **Hardware**: After ROS2
- ⏳ **Integration**: Final step

**Ready to flash?** Follow Step 3.1-3.3 and let me know when Ubuntu Server is running! 🎯 