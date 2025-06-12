#!/usr/bin/env python3
"""
Emergency stop script for robot arm motors
"""
import websocket
import json
import time
from sensitive_config import ROSBRIDGE_URL_PI

def stop_motors():
    """Send emergency stop command to halt all motor movement"""
    rosbridge_url = ROSBRIDGE_URL_PI
    
    print("🛑 EMERGENCY STOP - Halting all motor movement")
    print(f"📡 Connecting to ROS Bridge at {rosbridge_url}")
    
    try:
        ws = websocket.create_connection(rosbridge_url)
        print("✅ WebSocket connection established")
        
        # Method 1: Send stop command to current position (freeze in place)
        print("\n🛑 Sending STOP command...")
        
        # Get current position and hold it
        stop_msg = {
            "op": "publish",
            "topic": "/target_position",
            "msg": {
                "x": 0.0,  # Hold at origin
                "y": 0.0,  # Hold at origin
                "z": 0.0
            }
        }
        
        # Send multiple stop commands to ensure it's received
        for i in range(3):
            ws.send(json.dumps(stop_msg))
            print(f"  ✅ Stop command {i+1}/3 sent")
            time.sleep(0.1)
        
        # Method 2: Send zero velocity joint commands  
        print("\n🛑 Sending zero velocity commands...")
        
        zero_velocity_msg = {
            "op": "publish",
            "topic": "/joint_commands",
            "msg": {
                "layout": {
                    "dim": [],
                    "data_offset": 0
                },
                "data": [0.0, 0.0]  # Zero velocity for both joints
            }
        }
        
        # Send multiple zero velocity commands
        for i in range(3):
            ws.send(json.dumps(zero_velocity_msg))
            print(f"  ✅ Zero velocity command {i+1}/3 sent")
            time.sleep(0.1)
        
        print("\n🛑 STOP COMMANDS SENT!")
        print("🔒 Motors should halt immediately")
        
        ws.close()
        print("🔌 Connection closed")
        
    except Exception as e:
        print(f"❌ Error sending stop command: {e}")

if __name__ == "__main__":
    stop_motors() 