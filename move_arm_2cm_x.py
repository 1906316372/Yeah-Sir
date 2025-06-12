#!/usr/bin/env python3
"""
Script to move robot arm 2cm in x direction
"""
import sys
import json
import os
from sensitive_config import RASPBERRY_PI_IP, ROSBRIDGE_PORT, UBUNTU_IP

# Add the ros-mcp-server path to import the WebSocket manager
import os
ros_mcp_path = os.path.expanduser('~/GOOD_Boy/ros-mcp-server')
sys.path.append(ros_mcp_path)

try:
    from utils.websocket_manager import WebSocketManager
    
    # Network configuration
    ROSBRIDGE_IP = RASPBERRY_PI_IP
    LOCAL_IP = UBUNTU_IP
    
    print("🤖 Moving robot arm 2cm in X direction...")
    print(f"📡 Connecting to ROS Bridge at {ROSBRIDGE_IP}:{ROSBRIDGE_PORT}")
    
    # Create WebSocket manager and connect
    ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
    ws_manager.connect()
    
    if ws_manager.ws is None:
        print("❌ Failed to connect to ROS Bridge WebSocket")
        sys.exit(1)
    
    # Prepare the movement command for relative movement (2cm in x direction)
    command_data = {
        "type": "move_relative",
        "dx": 2.0,  # 2cm in x direction
        "dy": 0.0   # No movement in y direction
    }
    
    # Create ROS message to publish to /arm_commands topic
    ros_message = {
        "op": "publish",
        "topic": "/arm_commands",
        "msg": {
            "data": json.dumps(command_data)
        }
    }
    
    # Alternatively, try direct position command to /target_position
    # Get current position first (assuming starting from a known position)
    # For now, let's try a direct position command
    target_position_msg = {
        "op": "publish", 
        "topic": "/target_position",
        "msg": {
            "x": 2.0,  # 2cm from origin in x direction
            "y": 0.0,  # At origin in y direction
            "z": 0.0   # 2D arm, so z=0
        }
    }
    
    print(f"📤 Sending command: {command_data}")
    
    # Send the arm command
    ws_manager.send(ros_message)
    print("✅ Successfully sent movement command to /arm_commands")
    
    # Also send to target_position topic as backup
    ws_manager.send(target_position_msg)
    print("✅ Successfully sent position command to /target_position")
    
    print("🎯 Robot arm should now move 2cm in the X direction")
    
    # Close connection
    ws_manager.close()
    print("🔌 WebSocket connection closed")
    
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Please ensure you're in the correct directory with the ros-mcp-server")
    sys.exit(1)
except Exception as e:
    print(f"❌ Error executing movement command: {e}")
    sys.exit(1) 