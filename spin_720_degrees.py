#!/usr/bin/env python3
"""
Script to spin robot arm joint 720 degrees (2 full rotations)
"""
import websocket
import json
import time
from sensitive_config import ROSBRIDGE_URL_PI

def spin_joint_720_degrees():
    """Send command to spin joint 720 degrees using unlocked arm"""
    rosbridge_url = ROSBRIDGE_URL_PI
    
    print("🔄 Spinning joint 720° (2 full rotations) - UNLOCKED MODE")
    print(f"📡 Connecting to ROS Bridge at {rosbridge_url}")
    
    try:
        ws = websocket.create_connection(rosbridge_url)
        print("✅ WebSocket connection established")
        
        # Method 1: Direct position that forces large angle movement
        print("\n🎯 Sending 720° rotation command...")
        
        # Send to a position that requires 720-degree joint rotation
        # Using a far position that will cause the arm to rotate multiple times
        target_msg = {
            "op": "publish",
            "topic": "/target_position",
            "msg": {
                "x": 100.0,  # Very far position to force extreme rotation
                "y": 100.0,  # Very far position  
                "z": 0.0
            }
        }
        
        ws.send(json.dumps(target_msg))
        print("✅ Sent extreme position command: (100, 100) - This should cause 720° rotation")
        
        time.sleep(3)
        
        # Method 2: Try publishing directly to joint_commands with 720-degree angle
        print("\n🔄 Sending direct 720° joint command...")
        
        joint_cmd_msg = {
            "op": "publish",
            "topic": "/joint_commands", 
            "msg": {
                "layout": {
                    "dim": [],
                    "data_offset": 0
                },
                "data": [720.0, 0.0]  # 720° for joint1, 0° for joint2
            }
        }
        
        ws.send(json.dumps(joint_cmd_msg))
        print("✅ Sent direct joint command: Joint1 = 720°, Joint2 = 0°")
        
        time.sleep(3)
        
        # Method 3: Multiple incremental 180-degree rotations to reach 720°
        print("\n🔄 Sending incremental rotations to reach 720°...")
        
        incremental_angles = [180, 360, 540, 720]
        
        for angle in incremental_angles:
            print(f"  📍 Rotating to {angle}°...")
            
            incremental_msg = {
                "op": "publish",
                "topic": "/joint_commands",
                "msg": {
                    "layout": {
                        "dim": [],
                        "data_offset": 0
                    },
                    "data": [float(angle), 0.0]
                }
            }
            
            ws.send(json.dumps(incremental_msg))
            print(f"    ✅ Sent: Joint1 = {angle}°")
            time.sleep(2)  # Wait between increments
        
        print("\n🎉 720° rotation sequence complete!")
        print("🔄 The arm should have completed 2 full rotations (720°)")
        
        ws.close()
        print("🔌 Connection closed")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    spin_joint_720_degrees() 