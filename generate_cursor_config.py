#!/usr/bin/env python3
"""
生成cursor_mcp_config.json文件，使用敏感信息配置
"""
import json
import os
from sensitive_config import UBUNTU_IP, RASPBERRY_PI_IP, ROSBRIDGE_PORT

def generate_cursor_config():
    """生成cursor MCP配置文件"""
    
    # 获取当前用户的用户名
    username = os.environ.get('USER', 'user')
    
    config = {
        "mcpServers": {
            "ros-mcp-server": {
                "command": "uv",
                "args": [
                    "--directory",
                    f"/home/{username}/GOOD_Boy/ros-mcp-server",  # 动态用户名
                    "run",
                    "server.py"
                ],
                "env": {
                    "LOCAL_IP": UBUNTU_IP,
                    "ROSBRIDGE_IP": RASPBERRY_PI_IP,
                    "ROSBRIDGE_PORT": str(ROSBRIDGE_PORT)
                }
            }
        }
    }
    
    # 写入配置文件
    with open('cursor_mcp_config.json', 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"✅ 已生成cursor_mcp_config.json配置文件")
    print(f"   LOCAL_IP: {UBUNTU_IP}")
    print(f"   ROSBRIDGE_IP: {RASPBERRY_PI_IP}")
    print(f"   ROSBRIDGE_PORT: {ROSBRIDGE_PORT}")

if __name__ == "__main__":
    generate_cursor_config() 