#!/usr/bin/env python3
"""
更新~/.cursor/mcp.json文件，使用敏感信息配置
"""
import json
import os
from sensitive_config import UBUNTU_IP, RASPBERRY_PI_IP, ROSBRIDGE_PORT

def update_cursor_mcp():
    """更新cursor MCP配置文件"""
    
    # 获取当前用户的用户名
    username = os.environ.get('USER', 'user')
    cursor_config_path = os.path.expanduser('~/.cursor/mcp.json')
    
    config = {
        "mcpServers": {
            "ros-mcp-server": {
                "command": f"/home/{username}/GOOD_Boy/ros-mcp-server/.venv/bin/python3",
                "args": [
                    f"/home/{username}/GOOD_Boy/robot_arm_server.py"
                ],
                "env": {
                    "LOCAL_IP": UBUNTU_IP,
                    "ROSBRIDGE_IP": RASPBERRY_PI_IP,
                    "ROSBRIDGE_PORT": str(ROSBRIDGE_PORT)
                }
            }
        }
    }
    
    # 确保目录存在
    os.makedirs(os.path.dirname(cursor_config_path), exist_ok=True)
    
    # 写入配置文件
    with open(cursor_config_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"✅ 已更新 {cursor_config_path}")
    print(f"   LOCAL_IP: {UBUNTU_IP}")
    print(f"   ROSBRIDGE_IP: {RASPBERRY_PI_IP}")
    print(f"   ROSBRIDGE_PORT: {ROSBRIDGE_PORT}")
    print(f"   用户名: {username}")

if __name__ == "__main__":
    update_cursor_mcp() 