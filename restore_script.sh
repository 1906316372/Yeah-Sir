#!/bin/bash
# GOOD_Boy Robot Arm - Automated Restoration Script
# Run this script on your new Ubuntu machine after setting up the Raspberry Pi

set -e  # Exit on any error

echo "🚀 GOOD_Boy Robot Arm - Automated Restoration"
echo "=============================================="

# Configuration - Load from sensitive_config.py
eval $(python3 -c "
from sensitive_config import RASPBERRY_PI_IP, PI_USERNAME
print(f'PI_IP=\"{RASPBERRY_PI_IP}\"')
print(f'PI_USER=\"{PI_USERNAME}\"')
")

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "Do not run this script as root"
   exit 1
fi

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

print_status "Step 1: Installing system dependencies..."

# Update package manager
sudo apt update

# Install basic dependencies
sudo apt install -y \
    python3 \
    python3-pip \
    python3-websocket \
    openssh-client \
    curl \
    wget \
    git

print_success "System dependencies installed"

print_status "Step 2: Installing Python packages..."

# Install Python packages
pip3 install websocket-client

print_success "Python packages installed"

print_status "Step 3: Testing network connectivity..."

# Test connectivity to Raspberry Pi
if ping -c 1 $PI_IP >/dev/null 2>&1; then
    print_success "Raspberry Pi ($PI_IP) is reachable"
else
    print_warning "Cannot reach Raspberry Pi at $PI_IP"
    echo "Please verify:"
    echo "1. Raspberry Pi is powered on"
    echo "2. Network connection is working"  
    echo "3. IP address is correct in this script"
    read -p "Press Enter to continue anyway, or Ctrl+C to exit..."
fi

print_status "Step 4: Setting up SSH keys (if not already done)..."

# Check if SSH key exists
if [ ! -f ~/.ssh/id_rsa ]; then
    print_status "Generating SSH key..."
    ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""
    print_success "SSH key generated"
else
    print_success "SSH key already exists"
fi

# Test SSH connection
if ssh -o ConnectTimeout=5 -o BatchMode=yes $PI_USER@$PI_IP exit 2>/dev/null; then
    print_success "SSH connection to Pi working (passwordless)"
else
    print_warning "SSH connection requires password or setup"
    echo "To set up passwordless SSH, run:"
    echo "ssh-copy-id $PI_USER@$PI_IP"
    read -p "Press Enter to continue..."
fi

print_status "Step 5: Testing ROS Bridge connection..."

# Test if ROS Bridge is running
if curl -s --connect-timeout 5 http://$PI_IP:9090 >/dev/null 2>&1; then
    print_success "ROS Bridge WebSocket is running on Pi"
else
    print_warning "ROS Bridge not detected on Pi"
    echo "On the Raspberry Pi, run:"
    echo "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
fi

print_status "Step 6: Testing working MCP functions..."

# Test the working MCP functions
if [ -f "working_mcp_functions.py" ]; then
    print_status "Testing robot connection..."
    if python3 working_mcp_functions.py; then
        print_success "Robot connection test passed!"
    else
        print_warning "Robot connection test failed - check Pi setup"
    fi
else
    print_error "working_mcp_functions.py not found in current directory"
fi

print_status "Step 7: Setting up project structure..."

# Create commonly used directories
mkdir -p ~/robot_logs
mkdir -p ~/robot_scripts

print_success "Project structure created"

print_status "Step 8: Final system check..."

echo ""
echo "🎯 System Status Summary:"
echo "========================"

# Check Python
if command_exists python3; then
    PYTHON_VERSION=$(python3 --version)
    print_success "Python: $PYTHON_VERSION"
else
    print_error "Python3 not found"
fi

# Check websocket
if python3 -c "import websocket" 2>/dev/null; then
    print_success "WebSocket client: Available"
else
    print_error "WebSocket client: Missing"
fi

# Check network
if ping -c 1 $PI_IP >/dev/null 2>&1; then
    print_success "Pi Network: Reachable ($PI_IP)"
else
    print_warning "Pi Network: Not reachable"
fi

# Check SSH
if ssh -o ConnectTimeout=5 -o BatchMode=yes $PI_USER@$PI_IP exit 2>/dev/null; then
    print_success "SSH Access: Passwordless"
else
    print_warning "SSH Access: Requires password"
fi

# Check ROS Bridge
if curl -s --connect-timeout 5 http://$PI_IP:9090 >/dev/null 2>&1; then
    print_success "ROS Bridge: Running"
else
    print_warning "ROS Bridge: Not detected"
fi

echo ""
print_success "🎉 Restoration script completed!"
echo ""
echo "📋 Next Steps:"
echo "1. Ensure Raspberry Pi is running the robot software:"
echo "   ssh $PI_USER@$PI_IP"
echo "   cd ~/ros2_ws && source install/setup.bash"
echo "   ros2 launch robot_arm_controller robot_arm_launch.py"
echo ""
echo "2. Test basic movement:"
echo "   python3 simple_move_2cm_x.py"
echo ""
echo "3. For emergency stop:"
echo "   python3 -c \"from working_mcp_functions import stop_all_motion; stop_all_motion()\""
echo ""
echo "📖 For detailed instructions, see COMPLETE_BACKUP_README.md"
echo ""

exit 0 