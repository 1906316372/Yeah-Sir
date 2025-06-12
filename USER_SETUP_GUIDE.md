# 🤖 机器人臂控制项目 - 用户配置指南

> **版本**: v1.0  
> **更新日期**: 2025年1月1日  
> **适用平台**: Ubuntu 22.04, Raspberry Pi 4B

## 📋 概述

本项目是一个基于ROS2的2自由度机器人臂控制系统，支持通过自然语言命令控制机器人臂运动。项目采用了安全化配置管理，所有敏感信息（IP地址、用户名等）都需要用户根据自己的环境进行配置。

## 🎯 快速开始

### 1. 克隆项目
```bash
git clone <your-repository-url>
cd GOOD_Boy_github
```

### 2. 配置敏感信息
```bash
# 复制配置模板
cp sensitive_config.py.example sensitive_config.py

# 编辑配置文件
nano sensitive_config.py  # 或使用其他编辑器
```

### 3. 运行配置脚本
```bash
# 生成Cursor MCP配置
python3 generate_cursor_config.py

# 更新Cursor配置
python3 update_cursor_mcp.py
```

## ⚙️ 详细配置步骤

### 步骤1: 网络配置

在 `sensitive_config.py` 中修改以下配置：

```python
# ========================================
# 网络配置 - 必须修改
# ========================================

# Ubuntu开发机器IP地址（运行Cursor和MCP服务器的机器）
UBUNTU_IP = "192.168.1.100"  # 👈 修改为您的Ubuntu机器实际IP

# 树莓派IP地址（运行ROS2和机器人控制的设备）
RASPBERRY_PI_IP = "192.168.1.101"  # 👈 修改为您的树莓派实际IP

# Docker环境IP（通常保持默认值即可）
DOCKER_IP = "172.18.188.48"  # 👈 如果使用Docker可保持默认
```

**如何获取IP地址？**

在Ubuntu机器上：
```bash
# 获取Ubuntu机器IP
hostname -I | awk '{print $1}'
# 或者
ip addr show | grep "inet " | grep -v 127.0.0.1
```

在树莓派上：
```bash
# SSH到树莓派后执行
hostname -I | awk '{print $1}'
```

### 步骤2: 用户配置

```python
# ========================================
# 用户配置 - 必须修改
# ========================================

# 树莓派用户名（SSH登录时使用的用户名）
PI_USERNAME = "pi"  # 👈 修改为您的树莓派用户名
```

**如何确认用户名？**
```bash
# 在树莓派上执行
whoami
```

### 步骤3: WiFi配置（可选）

如果您的脚本需要配置WiFi，请修改：

```python
# ========================================
# WiFi配置 - 可选修改
# ========================================

# WiFi网络名称
WIFI_SSID = "YourWiFiName"  # 👈 修改为您的WiFi名称

# WiFi密码  
WIFI_PASSWORD = "YourWiFiPassword"  # 👈 修改为您的WiFi密码
```

### 步骤4: 验证配置

运行配置验证：
```bash
python3 sensitive_config.py
```

您应该看到类似输出：
```
🔧 敏感信息配置验证
========================================
✅ IP地址格式验证通过

📋 当前配置:
   Ubuntu IP: 192.168.1.100
   树莓派 IP: 192.168.1.101
   Docker IP: 172.18.188.48
   用户名: pi
   ROS桥接URL: ws://192.168.1.101:9090
   SSH命令: ssh pi@192.168.1.101

请确认以上配置是否正确！
```

## 🔧 环境准备

### 系统要求

**Ubuntu机器（开发机）:**
- Ubuntu 22.04 LTS
- Python 3.8+
- Cursor IDE
- Docker (可选)

**树莓派:**
- Ubuntu Server 22.04 LTS (ARM64)
- ROS2 Humble
- Python 3.8+

### 硬件要求

- **树莓派 4B** (推荐4GB以上内存)
- **舵机**: 2个数字舵机
- **舵机驱动板**: PCA9685或类似
- **杜邦线和面包板**
- **电源**: 5V 2A以上

## 🚀 部署选项

### 选项1: 标准部署（推荐）

1. **配置环境**
```bash
# 安装Python依赖
pip3 install -r requirements.txt

# 配置Cursor MCP
python3 generate_cursor_config.py
python3 update_cursor_mcp.py
```

2. **启动ROS桥接（在树莓派上）**
```bash
# SSH到树莓派
ssh PI_USERNAME@RASPBERRY_PI_IP

# 启动ROS桥接服务器
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. **启动MCP服务器（在Ubuntu上）**
```bash
cd ros-mcp-server
uv run robot_arm_server
```

### 选项2: Docker部署

1. **构建Docker镜像**
```bash
cd ros-mcp-server
docker build -f Dockerfile.improved -t ros-mcp-server:prod .
```

2. **运行Docker容器**
```bash
docker-compose up -d
```

## 🎮 使用方法

### 基本控制命令

在Cursor中，您可以使用自然语言命令：

```
// 基础运动
"Move arm 2cm to the right"
"Rotate base 90 degrees clockwise"
"Move to home position"

// 复杂动作
"Pick up object at position (10, 5)"
"Draw a circle with 5cm radius" 
"Test full range of motion"

// 状态查询
"Get current arm position"
"Show available ROS topics"
"Check system status"
```

### 程序化控制

直接运行Python脚本：

```bash
# 简单运动测试
python3 simple_move_2cm_x.py

# 旋转测试
python3 test_360_rotation.py

# 停止所有运动
python3 stop_motor.py
```

## 🐛 故障排除

### 常见问题

#### 1. 连接问题
**问题**: 无法连接到树莓派
```bash
# 检查网络连通性
ping RASPBERRY_PI_IP

# 检查SSH连接
ssh PI_USERNAME@RASPBERRY_PI_IP

# 检查ROS桥接端口
telnet RASPBERRY_PI_IP 9090
```

#### 2. MCP服务器问题
**问题**: Cursor无法连接到MCP服务器
```bash
# 检查MCP配置
cat ~/.config/Cursor/User/cursor_mcp_config.json

# 重新生成配置
python3 generate_cursor_config.py
python3 update_cursor_mcp.py

# 重启Cursor
```

#### 3. ROS桥接问题
**问题**: ROS桥接服务器无法启动
```bash
# 在树莓派上检查ROS2安装
source /opt/ros/humble/setup.bash
ros2 --version

# 检查rosbridge包
ros2 pkg list | grep rosbridge

# 手动启动rosbridge
ros2 run rosbridge_server rosbridge_websocket
```

#### 4. 硬件连接问题
**问题**: 舵机无响应
```bash
# 检查I2C连接
sudo i2cdetect -y 1

# 检查GPIO权限
sudo usermod -a -G gpio PI_USERNAME

# 测试舵机驱动
python3 -c "from adafruit_servokit import ServoKit; kit = ServoKit(channels=16); kit.servo[0].angle = 90"
```

### 日志查看

```bash
# MCP服务器日志
tail -f ros-mcp-server/logs/mcp.log

# ROS桥接日志（在树莓派上）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml --screen

# 系统日志
journalctl -f -u your-service-name
```

## 📁 项目结构

```
GOOD_Boy_github/
├── sensitive_config.py              # 🔧 主配置文件（需要配置）
├── sensitive_config.py.example      # 📋 配置模板
├── generate_cursor_config.py        # 🛠️ 配置生成脚本
├── update_cursor_mcp.py             # 🔄 配置更新脚本
├── 
├── ros-mcp-server/                  # 🤖 MCP服务器
│   ├── server.py                   # 主服务器文件
│   ├── Dockerfile.improved         # 生产环境Docker文件
│   ├── docker-compose.yml          # Docker编排文件
│   └── setup_cursor_mcp.sh         # 快速配置脚本
│
├── pi_ros2_workspace/               # 🔩 ROS2工作空间
│   └── src/robot_arm_controller/   # 机器人臂控制包
│
├── 控制脚本/
│   ├── robot_arm_server.py         # 🎮 主控制服务器
│   ├── simple_move_2cm_x.py        # ➡️ 简单X轴运动
│   ├── test_360_rotation.py        # 🔄 360度旋转测试
│   ├── stop_motor.py               # ⏹️ 紧急停止
│   └── working_mcp_functions.py    # 🔧 MCP功能库
│
└── 文档/
    ├── USER_SETUP_GUIDE.md         # 📖 本文档
    ├── COMPLETE_SETUP_GUIDE.md     # 📚 完整安装指南
    ├── HARDWARE_CONFIGURATION.md   # 🔌 硬件配置说明
    └── modify_recording.md         # 📝 修改记录
```

## 🔒 安全注意事项

1. **配置文件安全**: `sensitive_config.py` 包含敏感信息，已加入 `.gitignore`，请勿提交到版本控制
2. **网络安全**: 确保ROS桥接端口(9090)仅在可信网络中开放
3. **SSH安全**: 使用密钥认证而非密码认证
4. **硬件安全**: 在测试时确保机器人臂周围无障碍物

## 📞 获取帮助

### 文档资源
- 📖 [完整安装指南](COMPLETE_SETUP_GUIDE.md)
- 🔌 [硬件配置说明](HARDWARE_CONFIGURATION.md)
- 🐳 [Docker部署指南](DOCKER_SETUP.md)
- 🍓 [树莓派设置指南](RASPBERRY_PI_SETUP.md)

### 检查清单

配置完成后，请确认以下项目：

- [ ] ✅ 已复制并修改 `sensitive_config.py`
- [ ] ✅ IP地址配置正确
- [ ] ✅ 用户名配置正确
- [ ] ✅ 能够SSH连接到树莓派
- [ ] ✅ ROS2在树莓派上正常运行
- [ ] ✅ 已生成Cursor MCP配置
- [ ] ✅ 硬件连接正确
- [ ] ✅ 基本控制命令测试通过

### 版本兼容性

| 组件 | 推荐版本 | 最低版本 |
|------|----------|----------|
| Ubuntu | 22.04 LTS | 20.04 LTS |
| Python | 3.10+ | 3.8+ |
| ROS2 | Humble | Galactic |
| Cursor | 最新版 | 0.38+ |
| Docker | 24.0+ | 20.10+ |

---

🎉 **恭喜！** 按照本指南配置完成后，您就可以开始使用自然语言控制您的机器人臂了！

如果遇到问题，请检查故障排除部分或查看相关文档。祝您使用愉快！ 🤖✨ 