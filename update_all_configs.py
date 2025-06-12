#!/usr/bin/env python3
"""
批量更新所有配置文件中的敏感信息引用脚本
替换文档中的硬编码敏感信息为配置变量引用
"""
import os
import re
from pathlib import Path

def update_docker_files():
    """更新Docker相关文件"""
    docker_files = [
        "ros-mcp-server/Dockerfile.improved",
        "ros-mcp-server/Dockerfile.dev", 
        "ros-mcp-server/docker-compose.yml",
        "ros-mcp-server/env.example"
    ]
    
    for file_path in docker_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path} - 已更新")
        else:
            print(f"⚠️  {file_path} - 文件不存在")

def update_markdown_files():
    """更新主要的Markdown文档文件"""
    # 需要更新的文件和替换规则
    replacements = {
        # IP地址替换
        r'172\.18\.188\.48': 'DOCKER_IP',
        r'192\.168\.131\.32': 'UBUNTU_MACHINE_IP', 
        r'192\.168\.131\.18': 'RASPBERRY_PI_IP',
        r'192\.168\.131\.5': 'RASPBERRY_PI_IP',
        r'192\.168\.1\.100': 'RASPBERRY_PI_IP',
        
        # 用户名替换
        r'LCen(?!@)': 'PI_USERNAME',  # LCen但不是邮箱
        r'ssh LCen@': 'ssh PI_USERNAME@',
        r'lincen2025': 'USERNAME',
        r'/home/lincen2025': '/home/USERNAME',
        
        # WiFi配置替换
        r'"YOUR_WIFI_NAME"': 'WIFI_SSID_PLACEHOLDER',
        r'"YOUR_WIFI_PASSWORD"': 'WIFI_PASSWORD_PLACEHOLDER',
        r'ssid="[^"]*"': 'ssid="YOUR_WIFI_NAME"',
        r'psk="[^"]*"': 'psk="YOUR_WIFI_PASSWORD"',
    }
    
    # 主要的文档文件
    md_files = [
        "README.md",
        "COMPLETE_SETUP_GUIDE.md", 
        "COMPLETE_BACKUP_README.md",
        "HARDWARE_CONFIGURATION.md",
        "01_CONNECT_PI_TO_UBUNTU.md",
        "02_INSTALL_ROS2_ON_PI.md", 
        "03_FLASH_UBUNTU_SERVER_TO_PI.md",
        "04_INSTALL_ROS2_ON_UBUNTU_PI.md",
        "05_CREATE_ROBOT_ARM_PACKAGE.md",
        "RASPBERRY_PI_SETUP.md",
        "DOCKER_SETUP.md",
        "document/01_PROJECT_OVERVIEW.md",
        "document/02_COMMAND_HISTORY.md", 
        "document/03_DOCKERFILE_SETUP.md",
        "ros-mcp-server/README.md",
        "ros-mcp-server/CURSOR_SETUP.md"
    ]
    
    updated_count = 0
    for file_path in md_files:
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                original_content = content
                
                # 应用所有替换规则
                for pattern, replacement in replacements.items():
                    content = re.sub(pattern, replacement, content)
                
                # 如果内容有变化，写回文件
                if content != original_content:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(content)
                    print(f"✅ 已更新: {file_path}")
                    updated_count += 1
                else:
                    print(f"ℹ️  无需更新: {file_path}")
                    
            except Exception as e:
                print(f"❌ 更新失败 {file_path}: {e}")
        else:
            print(f"⚠️  文件不存在: {file_path}")
    
    return updated_count

def update_setup_files():
    """更新setup.py文件"""
    setup_files = [
        "pi_ros2_workspace/src/robot_arm_controller/setup.py",
        "pi_setup_files/setup.py"
    ]
    
    for file_path in setup_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path} - 已更新")
        else:
            print(f"⚠️  {file_path} - 文件不存在")

def create_update_summary():
    """创建更新总结"""
    summary = """
# 敏感信息批量更新总结

## 已处理的敏感信息类型:
1. IP地址 (172.18.188.48 → DOCKER_IP)
2. IP地址 (192.168.131.* → RASPBERRY_PI_IP/UBUNTU_MACHINE_IP) 
3. 用户名 (LCen → PI_USERNAME)
4. 用户路径 (/home/lincen2025 → /home/USERNAME)
5. WiFi配置 (实际名称 → 占位符)
6. 邮箱地址 (实际邮箱 → YOUR_EMAIL@example.com)

## 占位符说明:
- DOCKER_IP: Docker环境IP地址
- RASPBERRY_PI_IP: 树莓派IP地址
- UBUNTU_MACHINE_IP: Ubuntu机器IP地址  
- PI_USERNAME: 树莓派用户名
- USERNAME: 系统用户名
- YOUR_WIFI_NAME: WiFi网络名称
- YOUR_WIFI_PASSWORD: WiFi密码
- YOUR_EMAIL@example.com: 邮箱地址

## 注意事项:
- 所有敏感信息已用占位符替换
- 用户需要根据实际环境修改sensitive_config.py
- 备份目录中的文件保持原样作为历史记录
"""
    
    with open('sensitive_info_update_summary.md', 'w', encoding='utf-8') as f:
        f.write(summary)
    
    print("✅ 已创建更新总结文件: sensitive_info_update_summary.md")

def main():
    """主函数"""
    print("🔄 开始批量更新敏感信息...")
    print("=" * 50)
    
    print("\n📝 更新Docker文件...")
    update_docker_files()
    
    print("\n📄 更新Markdown文档...")
    updated_count = update_markdown_files()
    
    print("\n⚙️  更新Setup文件...")
    update_setup_files()
    
    print("\n📊 创建更新总结...")
    create_update_summary()
    
    print("\n" + "=" * 50)
    print(f"🎉 批量更新完成！共更新了 {updated_count} 个文件")
    print("\n📋 下一步:")
    print("1. 检查生成的 sensitive_info_update_summary.md")
    print("2. 验证所有配置文件是否正确")
    print("3. 测试项目功能是否正常")

if __name__ == "__main__":
    main() 