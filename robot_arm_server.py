from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
import re
import math
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState
from sensitive_config import UBUNTU_IP, RASPBERRY_PI_IP, ROSBRIDGE_PORT

# Network configuration
LOCAL_IP = UBUNTU_IP
ROSBRIDGE_IP = RASPBERRY_PI_IP

mcp = FastMCP("robot-arm-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)

# Robot arm specific topics
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/camera/image_raw")
jointstate = JointState(ws_manager, topic="/joint_states")

# Robot arm command topic for high-level commands
def publish_arm_command(command_data):
    """Publish structured command to robot arm"""
    try:
        msg = {
            "op": "publish",
            "topic": "/arm_commands",
            "msg": {
                "data": json.dumps(command_data)
            }
        }
        ws_manager.ws.send(json.dumps(msg))
        return True
    except Exception as e:
        print(f"Error publishing arm command: {e}")
        return False

@mcp.tool()
def get_topics():
    """Get list of available ROS topics"""
    topic_info = ws_manager.get_topics()
    ws_manager.close()

    if topic_info:
        topics, types = zip(*topic_info)
        return {
            "topics": list(topics),
            "types": list(types)
        }
    else:
        return "No topics found"

@mcp.tool()
def move_arm_relative(direction: str, distance: float, unit: str = "cm"):
    """
    Move robot arm relatively in specified direction
    
    Args:
        direction: Direction to move ('x', 'y', 'up', 'down', 'left', 'right', 'forward', 'backward')
        distance: Distance to move
        unit: Unit of measurement ('cm', 'mm', 'm')
    """
    # Convert distance to cm
    if unit == "mm":
        distance_cm = distance / 10
    elif unit == "m":
        distance_cm = distance * 100
    else:  # cm
        distance_cm = distance
    
    # Convert direction to x, y coordinates
    dx, dy = 0.0, 0.0
    
    direction_lower = direction.lower()
    if direction_lower in ['x', 'right']:
        dx = distance_cm
    elif direction_lower in ['-x', 'left']:
        dx = -distance_cm
    elif direction_lower in ['y', 'up', 'forward']:
        dy = distance_cm
    elif direction_lower in ['-y', 'down', 'backward']:
        dy = -distance_cm
    else:
        return f"Unknown direction: {direction}. Use 'x', 'y', 'up', 'down', 'left', 'right', 'forward', 'backward'"
    
    command = {
        "type": "move_relative",
        "dx": dx,
        "dy": dy
    }
    
    success = publish_arm_command(command)
    if success:
        return f"Moving arm {distance}{unit} in {direction} direction (dx={dx}cm, dy={dy}cm)"
    else:
        return "Failed to send movement command"

@mcp.tool()
def move_arm_to_position(x: float, y: float, unit: str = "cm"):
    """
    Move robot arm to absolute position
    
    Args:
        x: X coordinate
        y: Y coordinate  
        unit: Unit of measurement ('cm', 'mm', 'm')
    """
    # Convert to cm
    if unit == "mm":
        x_cm = x / 10
        y_cm = y / 10
    elif unit == "m":
        x_cm = x * 100
        y_cm = y * 100
    else:  # cm
        x_cm = x
        y_cm = y
    
    command = {
        "type": "move_absolute",
        "x": x_cm,
        "y": y_cm
    }
    
    success = publish_arm_command(command)
    if success:
        return f"Moving arm to position ({x_cm}cm, {y_cm}cm)"
    else:
        return "Failed to send position command"

@mcp.tool()
def move_arm_joints(joint1_degrees: float, joint2_degrees: float):
    """
    Move robot arm joints to specific angles
    
    Args:
        joint1_degrees: Base joint angle in degrees
        joint2_degrees: Second joint angle in degrees
    """
    command = {
        "type": "move_joint",
        "joint1": joint1_degrees,
        "joint2": joint2_degrees
    }
    
    success = publish_arm_command(command)
    if success:
        return f"Moving joints to J1={joint1_degrees}°, J2={joint2_degrees}°"
    else:
        return "Failed to send joint command"

@mcp.tool()
def home_arm():
    """Move robot arm to home position (straight up)"""
    command = {"type": "home"}
    
    success = publish_arm_command(command)
    if success:
        return "Moving arm to home position"
    else:
        return "Failed to send home command"

@mcp.tool()
def stop_arm():
    """Stop robot arm movement immediately"""
    command = {"type": "stop"}
    
    success = publish_arm_command(command)
    if success:
        return "Arm movement stopped"
    else:
        return "Failed to send stop command"

@mcp.tool()
def parse_natural_movement(command_text: str):
    """
    Parse natural language movement commands and execute them
    
    Args:
        command_text: Natural language command like "move 2 cm along x direction"
    """
    command_lower = command_text.lower()
    
    # Extract distance and unit
    distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(cm|mm|m)', command_lower)
    if not distance_match:
        distance_match = re.search(r'(\d+(?:\.\d+)?)', command_lower)
        if distance_match:
            distance = float(distance_match.group(1))
            unit = "cm"  # default unit
        else:
            return "Could not extract distance from command"
    else:
        distance = float(distance_match.group(1))
        unit = distance_match.group(2)
    
    # Extract direction
    direction = None
    if any(word in command_lower for word in ['right', 'x direction', '+x']):
        direction = "right"
    elif any(word in command_lower for word in ['left', '-x']):
        direction = "left"
    elif any(word in command_lower for word in ['up', 'forward', 'y direction', '+y']):
        direction = "up"
    elif any(word in command_lower for word in ['down', 'backward', '-y']):
        direction = "down"
    
    if direction:
        return move_arm_relative(direction, distance, unit)
    
    # Check for position commands
    position_match = re.search(r'position\s*\(?\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\)?', command_lower)
    if position_match:
        x = float(position_match.group(1))
        y = float(position_match.group(2))
        return move_arm_to_position(x, y, unit)
    
    # Check for home command
    if any(word in command_lower for word in ['home', 'reset', 'center']):
        return home_arm()
    
    # Check for stop command
    if any(word in command_lower for word in ['stop', 'halt', 'freeze']):
        return stop_arm()
    
    return f"Could not parse command: {command_text}. Try commands like 'move 2 cm right' or 'go to position (10, 15)'"

# Keep existing MCP functions for compatibility
@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    """Send low-level twist commands (for compatibility)"""
    msg = twist.publish(linear, angular)
    ws_manager.close()
    
    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_image():
    """Get camera image from robot"""
    msg = image.subscribe()
    ws_manager.close()
    
    if msg is not None:
        return "Image data received and downloaded successfully"
    else:
        return "No image data received"

@mcp.tool()
def pub_jointstate(name: list[str], position: list[float], velocity: list[float], effort: list[float]):
    """Publish joint state commands"""
    msg = jointstate.publish(name, position, velocity, effort)
    ws_manager.close()
    if msg is not None:
        return "JointState message published successfully"
    else:
        return "No message published"

@mcp.tool()
def sub_jointstate():
    """Get current joint states"""
    msg = jointstate.subscribe()
    ws_manager.close()
    if msg is not None:
        return msg
    else:
        return "No JointState data received"

if __name__ == "__main__":
    mcp.run(transport="stdio") 