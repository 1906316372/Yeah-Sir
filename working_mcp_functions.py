#!/usr/bin/env python3
"""
Working MCP-style functions using proven WebSocket approach
These replicate the ROS MCP server tools with working WebSocket connections
"""
import websocket
import json
import time
from sensitive_config import ROSBRIDGE_URL_PI

# Network configuration
ROSBRIDGE_URL = ROSBRIDGE_URL_PI

def pub_twist(linear, angular):
    """Publish twist message - equivalent to mcp_ros-mcp-server_pub_twist"""
    print(f"🔄 Publishing twist: linear={linear}, angular={angular}")
    
    try:
        ws = websocket.create_connection(ROSBRIDGE_URL)
        
        twist_msg = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {
                    "x": linear[0] if len(linear) > 0 else 0.0,
                    "y": linear[1] if len(linear) > 1 else 0.0,
                    "z": linear[2] if len(linear) > 2 else 0.0
                },
                "angular": {
                    "x": angular[0] if len(angular) > 0 else 0.0,
                    "y": angular[1] if len(angular) > 1 else 0.0,
                    "z": angular[2] if len(angular) > 2 else 0.0
                }
            }
        }
        
        ws.send(json.dumps(twist_msg))
        print("✅ Twist message sent successfully")
        ws.close()
        return True
        
    except Exception as e:
        print(f"❌ Error sending twist: {e}")
        return False

def pub_jointstate(name, position, velocity, effort):
    """Publish joint state message - equivalent to mcp_ros-mcp-server_pub_jointstate"""
    print(f"🤖 Publishing joint state: {name} -> positions={position}")
    
    try:
        ws = websocket.create_connection(ROSBRIDGE_URL)
        
        joint_state_msg = {
            "op": "publish",
            "topic": "/joint_commands",
            "msg": {
                "layout": {
                    "dim": [],
                    "data_offset": 0
                },
                "data": position  # Direct position values
            }
        }
        
        ws.send(json.dumps(joint_state_msg))
        print("✅ Joint state sent successfully")
        ws.close()
        return True
        
    except Exception as e:
        print(f"❌ Error sending joint state: {e}")
        return False

def stop_all_motion():
    """Emergency stop all motion"""
    print("🛑 EMERGENCY STOP - Using working MCP functions")
    
    # Method 1: Stop via twist (zero velocity)
    success1 = pub_twist([0, 0, 0], [0, 0, 0])
    
    # Method 2: Stop via joint commands (zero position)
    success2 = pub_jointstate(["joint1", "joint2"], [0, 0], [0, 0], [0, 0])
    
    # Method 3: Direct position stop
    try:
        ws = websocket.create_connection(ROSBRIDGE_URL)
        stop_msg = {
            "op": "publish",
            "topic": "/target_position",
            "msg": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        ws.send(json.dumps(stop_msg))
        ws.close()
        success3 = True
        print("✅ Direct position stop sent")
    except Exception as e:
        print(f"❌ Direct stop failed: {e}")
        success3 = False
    
    if success1 or success2 or success3:
        print("🔒 Stop commands sent successfully!")
        return True
    else:
        print("❌ All stop methods failed")
        return False

def get_topics():
    """Get ROS topics - equivalent to mcp_ros-mcp-server_get_topics"""
    print("📋 Getting ROS topics...")
    
    try:
        ws = websocket.create_connection(ROSBRIDGE_URL)
        
        topics_msg = {
            "op": "call_service",
            "service": "/rosapi/topics",
            "id": "get_topics_request"
        }
        
        ws.send(json.dumps(topics_msg))
        response = ws.recv()
        ws.close()
        
        data = json.loads(response)
        if "values" in data:
            topics = data["values"].get("topics", [])
            types = data["values"].get("types", [])
            print(f"✅ Found {len(topics)} topics")
            return {"topics": topics, "types": types}
        else:
            print("❌ No topics data in response")
            return None
            
    except Exception as e:
        print(f"❌ Error getting topics: {e}")
        return None

# Test the functions
if __name__ == "__main__":
    print("🧪 Testing working MCP functions...")
    
    # Test 1: Get topics
    print("\n1. Testing get_topics:")
    topics = get_topics()
    
    # Test 2: Stop motion
    print("\n2. Testing stop_all_motion:")
    stop_success = stop_all_motion()
    
    print(f"\n🎯 Test results: Topics={'✅' if topics else '❌'}, Stop={'✅' if stop_success else '❌'}") 