#!/usr/bin/env python3
"""
Simple script to move robot arm 2cm in x direction using websocket-client
"""
import websocket
import json
import time
from sensitive_config import ROSBRIDGE_URL_PI

def move_arm_2cm_x():
    # Network configuration
    rosbridge_url = ROSBRIDGE_URL_PI
    
    print("🤖 Moving robot arm 2cm in X direction...")
    print(f"📡 Connecting to ROS Bridge at {rosbridge_url}")
    
    try:
        # Create WebSocket connection
        ws = websocket.create_connection(rosbridge_url)
        print("✅ WebSocket connection established")
        
        # Method 1: Try publishing to /target_position (absolute position)
        # This assumes we want to move to position (2, 0) in cm
        target_position_msg = {
            "op": "publish",
            "topic": "/target_position", 
            "msg": {
                "x": 2.0,  # 2cm from origin in x direction
                "y": 0.0,  # At origin in y direction  
                "z": 0.0   # 2D arm, so z=0
            }
        }
        
        print("📤 Sending target position command...")
        ws.send(json.dumps(target_position_msg))
        print(f"✅ Sent: Move to position (2, 0) cm")
        
        # Wait a moment
        time.sleep(0.5)
        
        # Method 2: Try publishing to /arm_commands (if available)
        relative_command = {
            "type": "move_relative",
            "dx": 2.0,  # 2cm in x direction
            "dy": 0.0   # No movement in y
        }
        
        arm_command_msg = {
            "op": "publish",
            "topic": "/arm_commands",
            "msg": {
                "data": json.dumps(relative_command)
            }
        }
        
        print("📤 Sending relative movement command...")
        ws.send(json.dumps(arm_command_msg))
        print(f"✅ Sent: Relative move +2cm in X")
        
        print("🎯 Commands sent successfully! Robot arm should move 2cm in X direction")
        
        # Close connection
        ws.close()
        print("🔌 WebSocket connection closed")
        
    except websocket.WebSocketException as e:
        print(f"❌ WebSocket error: {e}")
    except ConnectionRefusedError:
        print("❌ Connection refused. Is ROS Bridge running on the Pi?")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    move_arm_2cm_x() 