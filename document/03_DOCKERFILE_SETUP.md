# Dockerfile Setup - Robot Arm Project

## 🎯 **Overview**
This document provides Docker containerization for the 2-DOF Robot Arm Control Project setup.

---

## 📋 **Dockerfile for Ubuntu MCP Server**

```dockerfile
# Dockerfile for Ubuntu MCP Server
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LOCAL_IP=UBUNTU_MACHINE_IP
ENV ROSBRIDGE_IP=RASPBERRY_PI_IP
ENV ROSBRIDGE_PORT=9090

# Update system and install dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    curl \
    wget \
    git \
    nano \
    htop \
    python3 \
    python3-pip \
    build-essential \
    ca-certificates \
    gnupg \
    lsb-release \
    locales

# Set up locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install UV package manager
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.cargo/bin:$PATH"

# Set working directory
WORKDIR /app

# Clone ROS MCP Server repository
RUN git clone https://github.com/lpigeon/ros-mcp-server.git ros-mcp-server

# Setup Python environment
WORKDIR /app/ros-mcp-server
RUN /root/.cargo/bin/uv sync

# Create MCP configuration directory
RUN mkdir -p /root/.cursor

# Copy MCP configuration
COPY mcp.json /root/.cursor/mcp.json

# Create network configuration
RUN cat > network_config.txt << EOF
# Network Configuration for Robot Arm Project
# Updated: $(date)

Ubuntu Machine:
- IP Address: UBUNTU_MACHINE_IP
- Hostname: ubuntu-mcp-server

Raspberry Pi (Ubuntu Server):
- IP Address: RASPBERRY_PI_IP
- Hostname: ubuntu-pi
- Username: PI_USERNAME
- SSH Command: ssh PI_USERNAME@RASPBERRY_PI_IP
- OS: Ubuntu Server 22.04.5 LTS ARM64

ROS Bridge Configuration:
- Pi runs ROS Bridge on: ws://RASPBERRY_PI_IP:9090
- Ubuntu MCP Server connects to: ws://RASPBERRY_PI_IP:9090
EOF

# Expose port for MCP server
EXPOSE 8080

# Start MCP server
CMD ["/root/.cargo/bin/uv", "run", "robot_arm_server"]
```

---

## 📋 **Dockerfile for Raspberry Pi ROS2 Setup**

```dockerfile
# Dockerfile for Raspberry Pi ROS2 (ARM64)
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DOMAIN_ID=0

# Update system and install dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    curl \
    wget \
    git \
    nano \
    htop \
    software-properties-common \
    gnupg \
    lsb-release \
    locales \
    python3 \
    python3-pip \
    build-essential \
    ca-certificates

# Set up locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
RUN apt-get update

# Install ROS2 Humble
RUN apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-rosbridge-server \
    ros-humble-rosbridge-library

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install hardware libraries
RUN pip3 install \
    adafruit-circuitpython-servokit \
    RPi.GPIO

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

# Create workspace
WORKDIR /ros2_ws

# Source ROS2 environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash

# Expose ROS Bridge port
EXPOSE 9090

# Default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
```

---

## 📋 **Docker Compose Configuration**

```yaml
# docker-compose.yml
version: '3.8'

services:
  mcp-server:
    build:
      context: .
      dockerfile: Dockerfile.mcp
    container_name: robot-arm-mcp
    ports:
      - "8080:8080"
    networks:
      - robot-network
    environment:
      - LOCAL_IP=UBUNTU_MACHINE_IP
      - ROSBRIDGE_IP=RASPBERRY_PI_IP
      - ROSBRIDGE_PORT=9090
    volumes:
      - ./config:/app/config
      - ~/.cursor:/root/.cursor
    restart: unless-stopped

  ros2-bridge:
    build:
      context: .
      dockerfile: Dockerfile.ros2
    container_name: robot-arm-ros2
    ports:
      - "9090:9090"
    networks:
      - robot-network
    environment:
      - ROS_DOMAIN_ID=0
    devices:
      - /dev/i2c-1:/dev/i2c-1
    privileged: true
    restart: unless-stopped

networks:
  robot-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

---

## 📋 **Supporting Configuration Files**

### **mcp.json**
```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/app/ros-mcp-server",
        "run",
        "robot_arm_server"
      ],
      "env": {
        "LOCAL_IP": "UBUNTU_MACHINE_IP",
        "ROSBRIDGE_IP": "RASPBERRY_PI_IP", 
        "ROSBRIDGE_PORT": "9090"
      }
    }
  }
}
```

### **.dockerignore**
```
.git
.gitignore
README.md
*.log
*.tmp
__pycache__
.pytest_cache
node_modules
.env
```

---

## 📋 **Build and Run Commands**

### **Build Images**
```bash
# Build MCP Server image
docker build -f Dockerfile.mcp -t robot-arm-mcp:latest .

# Build ROS2 image (for Pi)
docker build -f Dockerfile.ros2 -t robot-arm-ros2:latest .
```

### **Run with Docker Compose**
```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### **Manual Container Run**
```bash
# Run MCP Server
docker run -d \
  --name robot-arm-mcp \
  -p 8080:8080 \
  -e LOCAL_IP=UBUNTU_MACHINE_IP \
  -e ROSBRIDGE_IP=RASPBERRY_PI_IP \
  -e ROSBRIDGE_PORT=9090 \
  robot-arm-mcp:latest

# Run ROS2 Bridge (on Pi)
docker run -d \
  --name robot-arm-ros2 \
  -p 9090:9090 \
  --device /dev/i2c-1:/dev/i2c-1 \
  --privileged \
  -e ROS_DOMAIN_ID=0 \
  robot-arm-ros2:latest
```

---

## 📋 **Multi-Architecture Support**

### **BuildX for ARM64**
```bash
# Create builder instance
docker buildx create --name multi-arch-builder --use

# Build for multiple architectures
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f Dockerfile.ros2 \
  -t robot-arm-ros2:multi-arch \
  --push .
```

### **Cross-Platform Development**
```bash
# Build for Raspberry Pi from Ubuntu machine
docker buildx build \
  --platform linux/arm64 \
  -f Dockerfile.ros2 \
  -t robot-arm-ros2:arm64 \
  --load .
```

---

## 📋 **Production Deployment**

### **Docker Swarm Configuration**
```yaml
# docker-stack.yml
version: '3.8'

services:
  mcp-server:
    image: robot-arm-mcp:latest
    deploy:
      replicas: 1
      placement:
        constraints:
          - node.platform.arch == x86_64
    ports:
      - "8080:8080"
    networks:
      - robot-network

  ros2-bridge:
    image: robot-arm-ros2:latest
    deploy:
      replicas: 1
      placement:
        constraints:
          - node.platform.arch == aarch64
    ports:
      - "9090:9090"
    networks:
      - robot-network

networks:
  robot-network:
    driver: overlay
    attachable: true
```

### **Deploy Stack**
```bash
# Initialize swarm
docker swarm init

# Deploy stack
docker stack deploy -c docker-stack.yml robot-arm

# Scale services
docker service scale robot-arm_mcp-server=2
```

---

## 📋 **Health Checks and Monitoring**

### **Health Check Configuration**
```dockerfile
# Add to Dockerfile.ros2
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:9090 || exit 1

# Add to Dockerfile.mcp
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:8080/health || exit 1
```

### **Monitoring with Prometheus**
```yaml
# Add to docker-compose.yml
  prometheus:
    image: prom/prometheus:latest
    ports:
      - "9091:9090"
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
    networks:
      - robot-network

  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
    networks:
      - robot-network
```

---

## ✅ **Advantages of Docker Approach**

### **Benefits**
- ✅ **Consistent Environment**: Same setup across machines
- ✅ **Easy Deployment**: Single command deployment
- ✅ **Scalability**: Easy to scale components
- ✅ **Isolation**: Components isolated from host system
- ✅ **Version Control**: Dockerfile tracks configuration changes
- ✅ **Multi-Architecture**: Support for x86_64 and ARM64

### **Use Cases**
- **Development**: Consistent dev environment
- **Testing**: Isolated test environments
- **Production**: Scalable deployment
- **CI/CD**: Automated testing and deployment
- **Education**: Easy setup for students

---

**Last Updated**: June 2, 2025  
**Tested Platforms**: Ubuntu 22.04 (x86_64), Raspberry Pi 4B (ARM64)  
**Docker Version**: 24.0+
