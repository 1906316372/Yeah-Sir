# Docker Setup for ROS MCP Server

## 🐳 Updated Docker Configuration

All Docker files have been updated with the correct IP configuration for your Ubuntu system:

### 📡 Network Configuration
- **Local IP**: `DOCKER_IP`
- **ROS Bridge IP**: `DOCKER_IP` 
- **ROS Bridge Port**: `9090`

## 🚀 Deployment Options

### 1. Quick Development (Dockerfile.dev)
```bash
# Build and run development version
docker build -f Dockerfile.dev -t ros-mcp-server:dev .
docker run --network host ros-mcp-server:dev
```

### 2. Production Deployment (docker-compose)
```bash
# Copy environment configuration
cp env.example .env

# Build and run with docker-compose
docker-compose up --build

# Run in background
docker-compose up -d --build
```

### 3. Production with ROS Bridge
```bash
# Run with optional ROS Bridge server
docker-compose --profile with-rosbridge up --build
```

### 4. Multi-stage Production Build
```bash
# Build production image
docker build -f Dockerfile.improved -t ros-mcp-server:prod .

# Run with environment variables
docker run --network host \
  -e LOCAL_IP=DOCKER_IP \
  -e ROSBRIDGE_IP=DOCKER_IP \
  -e ROSBRIDGE_PORT=9090 \
  ros-mcp-server:prod
```

## 🔧 Updated Files

| File | Purpose | Changes |
|------|---------|---------|
| `Dockerfile.improved` | Production multi-stage | Updated IP environment variables |
| `Dockerfile.dev` | Development | Added IP environment variables |
| `docker-compose.yml` | Orchestration | Updated default IP values |
| `env.example` | Environment template | Updated with correct IPs |

## 🌐 Network Configuration

All Docker containers will use:
- **Host networking** for ROS communication
- **IP**: `DOCKER_IP` (your Ubuntu system IP)
- **Port**: `9090` for ROS Bridge communication

## 📋 Environment Variables

You can override the IP configuration:

```bash
# For docker run
docker run --network host \
  -e LOCAL_IP=YOUR_IP \
  -e ROSBRIDGE_IP=YOUR_ROS_IP \
  ros-mcp-server:dev

# For docker-compose (edit .env file)
LOCAL_IP=YOUR_IP
ROSBRIDGE_IP=YOUR_ROS_IP
ROSBRIDGE_PORT=9090
```

## 🔍 Testing Docker Setup

### Test MCP Server
```bash
# Test development build
docker build -f Dockerfile.dev -t ros-mcp-server:dev .
docker run --rm ros-mcp-server:dev python -c "from server import mcp; print('✅ MCP working')"

# Test production build  
docker build -f Dockerfile.improved -t ros-mcp-server:prod .
docker run --rm ros-mcp-server:prod python -c "from server import mcp; print('✅ MCP working')"
```

### Test with ROS Bridge
```bash
# Start ROS Bridge first
roslaunch rosbridge_server rosbridge_websocket.launch

# Then start MCP server
docker-compose up
```

## 🐛 Troubleshooting

### Network Issues
```bash
# Check if port 9090 is available
sudo netstat -tulpn | grep 9090

# Test ROS Bridge connectivity
curl -v ws://DOCKER_IP:9090
```

### Container Issues
```bash
# Check container logs
docker-compose logs ros-mcp-server

# Enter container for debugging
docker-compose exec ros-mcp-server bash
```

### IP Configuration
```bash
# Verify your system IP
hostname -I | awk '{print $1}'

# Update Docker files if IP changed
grep -r "DOCKER_IP" . --include="*.yml" --include="Dockerfile*"
```

## 🎯 Integration with Cursor

The Docker containers are configured to work with the Cursor MCP integration:

1. **Start ROS Bridge**: `roslaunch rosbridge_server rosbridge_websocket.launch`
2. **Start MCP Server**: `docker-compose up -d`
3. **Use Cursor**: Natural language commands will work through MCP

Your Docker setup is now synchronized with your local configuration! 🚀 