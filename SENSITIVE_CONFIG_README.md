# 敏感信息配置说明

## 概述

本项目已将所有敏感信息（IP地址、用户名、密码等）集中管理，提高安全性和可移植性。

## 快速设置

### 1. 创建敏感信息配置文件

复制并修改敏感信息配置文件：
   
```bash
cp sensitive_config.py.example sensitive_config.py
```

然后编辑 `sensitive_config.py` 文件，将其中的值修改为您的实际环境配置。

### 2. 配置示例

```python
# 网络配置 - 请修改为您的实际IP地址
UBUNTU_IP = "192.168.1.100"      # 您的Ubuntu开发机器IP
RASPBERRY_PI_IP = "192.168.1.101" # 您的树莓派IP
DOCKER_IP = "172.18.188.48"       # Docker环境IP（通常无需修改）

# 用户配置 - 请修改为您的实际用户名
PI_USERNAME = "pi"                # 您的树莓派用户名

# WiFi配置 - 请修改为您的实际WiFi信息
WIFI_SSID = "YourWiFiName"
WIFI_PASSWORD = "YourWiFiPassword"
```

### 3. 生成配置文件

运行以下命令生成所需的配置文件：

```bash
# 生成cursor配置文件
python3 generate_cursor_config.py

# 更新cursor MCP配置
python3 update_cursor_mcp.py
```

## 文件说明

### 核心配置文件

- **`sensitive_config.py`** - 主要敏感信息配置文件（需要用户创建）
- **`sensitive_config.py.example`** - 配置文件示例模板

### 配置生成脚本

- **`generate_cursor_config.py`** - 生成cursor_mcp_config.json
- **`update_cursor_mcp.py`** - 更新~/.cursor/mcp.json

### 已更新的文件

以下文件已更新为使用敏感信息配置：

#### Python脚本
- `working_mcp_functions.py`
- `robot_arm_server.py`
- `test_360_rotation.py`
- `stop_motor.py`
- `spin_720_degrees.py`
- `simple_move_2cm_x.py`
- `move_arm_2cm_x.py`
- `ros-mcp-server/server.py`

#### Shell脚本
- `restore_script.sh`

#### 文档文件
- `network_config.txt`
- `HARDWARE_CONFIGURATION.md`

## 安全注意事项

### ⚠️ 重要提醒

1. **不要提交敏感信息** - `sensitive_config.py` 已添加到 `.gitignore`
2. **检查提交内容** - 确保不要意外提交包含真实IP/密码的文件
3. **定期检查** - 确认敏感信息没有硬编码在新添加的文件中

### Git安全

`.gitignore` 文件已配置排除以下敏感文件：
- `sensitive_config.py`
- `cursor_mcp_config.json`
- `~/.cursor/mcp.json`

## 故障排除

### 常见问题

1. **导入错误**: 确保 `sensitive_config.py` 文件存在且格式正确
2. **网络连接失败**: 检查IP地址配置是否正确
3. **权限问题**: 确保脚本有执行权限

### 验证配置

运行以下命令验证配置：

```bash
# 测试敏感信息配置
python3 -c "from sensitive_config import *; print(f'Ubuntu: {UBUNTU_IP}, Pi: {RASPBERRY_PI_IP}')"

# 测试网络连接
ping $RASPBERRY_PI_IP

# 测试ROS桥接
curl -v ws://$RASPBERRY_PI_IP:9090
```

## 开发者指南

### 添加新的敏感信息

1. 在 `sensitive_config.py` 中添加新的配置项
2. 更新相关文件以使用配置项
3. 更新本README文件
4. 在 `modify_recording.md` 中记录更改

### 代码规范

- 使用 `from sensitive_config import VARIABLE_NAME` 导入配置
- 在注释中说明敏感信息的用途
- 避免在日志中输出敏感信息

## 支持

如果遇到问题，请检查：
1. `modify_recording.md` - 查看最近的修改记录
2. 相关文档文件中的占位符说明
3. 确认所有配置文件都已正确生成

---

**注意**: 本系统设计用于提高项目安全性。请务必在实际部署前仔细检查所有配置。 