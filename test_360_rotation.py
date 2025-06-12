#!/usr/bin/env python3
"""
Test script for 360-degree rotation of unlocked robot arm
"""
import websocket
import json
import time
from sensitive_config import ROSBRIDGE_URL_PI

def test_360_rotation():
    """Test various angles including full 360-degree rotations"""
    rosbridge_url = ROSBRIDGE_URL_PI
    
    print("🔓 Testing 360-degree rotation - UNLOCKED MODE")
    print(f"📡 Connecting to ROS Bridge at {rosbridge_url}")
    
    try:
        ws = websocket.create_connection(rosbridge_url)
        print("✅ WebSocket connection established")
        
        # Test cases for 360-degree rotation
        test_positions = [
            (0, 0, "Origin"),
            (5, 0, "5cm X-axis"),
            (0, 5, "5cm Y-axis"),
            (10, 10, "Diagonal 10,10"),
            (20, 0, "Extended 20cm X"),
            (0, 20, "Extended 20cm Y"),
            (-10, 0, "Negative X"),
            (0, -10, "Negative Y"),
            (30, 0, "Far X (beyond normal workspace)"),
            (0, 30, "Far Y (beyond normal workspace)"),
            (50, 0, "Very far X - Test extreme"),
            (0, 50, "Very far Y - Test extreme")
        ]
        
        print("\n🔄 Starting 360-degree rotation tests...")
        
        for i, (x, y, description) in enumerate(test_positions):
            print(f"\n📍 Test {i+1}: {description} -> Position ({x}, {y})")
            
            # Create target position message
            target_msg = {
                "op": "publish",
                "topic": "/target_position",
                "msg": {
                    "x": float(x),
                    "y": float(y),
                    "z": 0.0
                }
            }
            
            # Send command
            ws.send(json.dumps(target_msg))
            print(f"✅ Sent command: Move to ({x}, {y})")
            
            # Wait for movement
            time.sleep(2)
        
        print("\n🎯 Testing complete! The arm should have moved through various positions")
        print("🔓 All workspace limits have been removed - 360° rotation enabled!")
        
        # Final test - return to center
        print("\n🏠 Returning to center position...")
        center_msg = {
            "op": "publish",
            "topic": "/target_position",
            "msg": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            }
        }
        ws.send(json.dumps(center_msg))
        print("✅ Returned to center")
        
        ws.close()
        print("🔌 Connection closed")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    test_360_rotation() 