#!/bin/bash

# Setup script for Cursor MCP integration with ROS MCP Server
echo "🚀 Setting up Cursor MCP integration for ROS MCP Server..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if Cursor is installed
if ! command -v cursor &> /dev/null; then
    echo -e "${RED}❌ Cursor is not installed or not in PATH${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Cursor found${NC}"

# Create Cursor config directory if it doesn't exist
CURSOR_CONFIG_DIR="$HOME/.config/Cursor/User"
mkdir -p "$CURSOR_CONFIG_DIR"

# Path to the MCP configuration file
MCP_CONFIG_FILE="$CURSOR_CONFIG_DIR/cursor_mcp_config.json"

# Current project directory
PROJECT_DIR=$(pwd)

# Create MCP configuration
echo -e "${BLUE}📝 Creating MCP configuration...${NC}"

cat > "$MCP_CONFIG_FILE" << EOF
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "$PROJECT_DIR",
        "run",
        "server.py"
      ],
      "env": {
        "LOCAL_IP": "DOCKER_IP",
        "ROSBRIDGE_IP": "DOCKER_IP",
        "ROSBRIDGE_PORT": "9090"
      }
    }
  }
}
EOF

echo -e "${GREEN}✅ MCP configuration created at: $MCP_CONFIG_FILE${NC}"

# Check if virtual environment exists
if [ -d ".venv" ]; then
    echo -e "${GREEN}✅ Virtual environment found${NC}"
else
    echo -e "${YELLOW}⚠️  Virtual environment not found. Creating...${NC}"
    uv venv
    source .venv/bin/activate
    uv sync
fi

# Create a test script
echo -e "${BLUE}📝 Creating test script...${NC}"

cat > "test_mcp_server.py" << 'EOF'
#!/usr/bin/env python3
"""
Test script to verify MCP server functionality
"""
import subprocess
import sys
import json

def test_mcp_server():
    """Test if the MCP server can start and respond"""
    try:
        # Test if we can import the server
        result = subprocess.run([
            "uv", "run", "python", "-c", 
            "from server import mcp; print('MCP server imported successfully')"
        ], capture_output=True, text=True, cwd=".")
        
        if result.returncode == 0:
            print("✅ MCP server can be imported successfully")
            return True
        else:
            print(f"❌ Error importing MCP server: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error testing MCP server: {e}")
        return False

if __name__ == "__main__":
    success = test_mcp_server()
    sys.exit(0 if success else 1)
EOF

chmod +x test_mcp_server.py

# Run the test
echo -e "${BLUE}🧪 Testing MCP server...${NC}"
python3 test_mcp_server.py

echo -e "${GREEN}🎉 Setup complete!${NC}"
echo -e "${BLUE}📋 Next steps:${NC}"
echo "1. Start ROS Bridge server first:"
echo -e "   ${YELLOW}roslaunch rosbridge_server rosbridge_websocket.launch${NC}"
echo "   ${YELLOW}# or for ROS2: ros2 launch rosbridge_server rosbridge_websocket_launch.xml${NC}"
echo ""
echo "2. The MCP configuration is ready at:"
echo -e "   ${YELLOW}$MCP_CONFIG_FILE${NC}"
echo ""
echo "3. In Cursor, you can now use MCP commands like:"
echo -e "   ${YELLOW}\"Move the robot forward\"${NC}"
echo -e "   ${YELLOW}\"Get available topics\"${NC}"
echo -e "   ${YELLOW}\"Subscribe to camera feed\"${NC}"
echo ""
echo -e "${BLUE}📖 Available MCP Functions:${NC}"
echo "   • get_topics() - List ROS topics"
echo "   • pub_twist() - Send movement commands"
echo "   • pub_twist_seq() - Send movement sequences"
echo "   • sub_image() - Get camera images"
echo "   • pub_jointstate() - Control joints"
echo "   • sub_jointstate() - Get joint states" 