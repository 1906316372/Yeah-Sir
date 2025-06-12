# GOOD_Boy Robot Arm - Hardware Configuration Reference

## 🔧 Hardware Specifications

### Main Components
- **Raspberry Pi 4B**: Ubuntu Server 22.04.5 LTS
- **Stepper Motors**: 2x NEMA 11 with 13.73:1 planetary gearboxes
- **Drivers**: 2x A4988 stepper motor drivers
- **Arm Structure**: Wooden segments (15cm base + 10cm end effector)

### Precision Specifications
- **Base Steps/Revolution**: 3,200 (1.8° motor × 1/16 microstepping = 3,200 steps/rev)
- **Gearbox Ratio**: 13.73:1
- **Total Steps/Revolution**: 43,936 steps
- **Angular Precision**: 0.0082° per step
- **Maximum Speed**: Variable (software controlled)

## 📡 Network Configuration

### IP Addresses
```
Ubuntu Development Machine: UBUNTU_MACHINE_IP
Raspberry Pi (Robot): RASPBERRY_PI_IP
ROS Bridge WebSocket: ws://RASPBERRY_PI_IP:9090
```

### Network Services
- **SSH**: Port 22 (passwordless access configured)
- **ROS Bridge WebSocket**: Port 9090
- **ROS2 DDS**: Default ports (automatic discovery)

## 🔌 GPIO Connections (Raspberry Pi)

### Motor 0 (Base Joint) - A4988 Driver
```
GPIO Pin → A4988 Pin
GPIO 21  → STEP
GPIO 20  → DIR
GPIO 16  → ENABLE (optional)
```

### Motor 1 (End Effector Joint) - A4988 Driver
```
GPIO Pin → A4988 Pin
GPIO 19  → STEP
GPIO 13  → DIR
GPIO 6   → ENABLE (optional)
```

### A4988 Driver Configuration
```
MS1, MS2, MS3 → Configure microstepping
VDD → 3.3V or 5V logic supply
VMOT → Motor power supply (12V recommended)
GND → Common ground
1A, 1B, 2A, 2B → Motor coil connections
```

### Power Supply Requirements
- **Logic Power**: 5V (from Pi)
- **Motor Power**: 12V DC, minimum 2A per driver
- **Total Power**: ~50W under full load

## 🎛️ Software Configuration

### ROS2 Topics
```
/target_position    - geometry_msgs/Point (X, Y, Z coordinates)
/joint_commands     - std_msgs/Float64MultiArray (joint angles)
/cmd_vel           - geometry_msgs/Twist (velocity commands)
/arm_commands      - std_msgs/String (text commands)
```

### Key Parameters
```python
# Arm geometry
L1 = 15.0  # Base segment length (cm)
L2 = 10.0  # End effector segment length (cm)

# Motor configuration
STEPS_PER_REV = 43936  # Total steps per revolution
STEP_ANGLE = 0.0082    # Degrees per step
MAX_SPEED = 1000       # Steps per second (adjustable)

# Workspace (UNLOCKED - no limits)
MIN_REACH = 0          # Minimum reach (unlimited)
MAX_REACH = ∞          # Maximum reach (unlimited)
ANGLE_RANGE = [-999°, +999°]  # Unlimited rotation
```

### Control Modes

#### 1. Position Control (Primary)
- **Input**: Target (X, Y) coordinates
- **Processing**: Inverse kinematics → joint angles → motor steps
- **Feedback**: Step counting with position confirmation

#### 2. Joint Control (Direct)
- **Input**: Joint angles in degrees
- **Processing**: Direct angle → motor steps conversion
- **Range**: Unlimited (-999° to +999°)

#### 3. Velocity Control
- **Input**: Linear/angular velocities
- **Processing**: Real-time velocity → step rate conversion
- **Safety**: Automatic deceleration and stopping

## 🔄 System States

### Normal Operation States
1. **INIT**: GPIO initialization, motor homing
2. **READY**: Waiting for commands
3. **MOVING**: Executing movement commands
4. **HOLDING**: Maintaining position
5. **STOPPED**: Emergency stop activated

### Error States
- **CONNECTION_ERROR**: Network/ROS communication failure
- **MOTOR_ERROR**: Motor driver or hardware fault
- **LIMIT_ERROR**: (Disabled in unlocked mode)
- **EMERGENCY_STOP**: Manual or automatic safety stop

## 🛡️ Safety Features

### Software Safety
- **Emergency stop functions**: Multiple redundant stop methods
- **Velocity limiting**: Configurable maximum speeds
- **Acceleration control**: Smooth acceleration/deceleration
- **Position validation**: Real-time position tracking

### Hardware Safety
- **Enable pins**: Hardware-level motor disable capability
- **Current limiting**: A4988 current adjustment potentiometers
- **Thermal protection**: Automatic thermal shutdown on drivers
- **Power monitoring**: Supply voltage monitoring

### Emergency Procedures
```bash
# Software Emergency Stop (Primary)
python3 -c "from working_mcp_functions import stop_all_motion; stop_all_motion()"

# Process Kill (Secondary)
ssh PI_USERNAME@RASPBERRY_PI_IP "pkill -f robot_arm"

# Hardware Emergency Stop (Ultimate)
# Disconnect power to motor drivers
```

## 📊 Performance Characteristics

### Movement Specifications
- **Angular Resolution**: 0.0082° per step
- **Repeatability**: ±0.01° (limited by mechanical backlash)
- **Maximum Speed**: ~1000 steps/sec (configurable)
- **Acceleration**: Configurable ramp profiles

### Workspace Characteristics
- **Physical Reach**: 5cm (minimum) to 25cm (maximum arm extension)
- **Software Limits**: DISABLED (unlimited rotation enabled)
- **Rotation Range**: Unlimited (multiple full rotations possible)
- **Position Accuracy**: High precision with step counting feedback

## 🔧 Calibration Parameters

### Motor Calibration
```python
# Steps per revolution verification
FULL_ROTATION_STEPS = 43936

# Home position definition
HOME_POSITION = {
    'motor_0': 0,      # Base joint at 0°
    'motor_1': -90     # End effector at -90° (vertical down)
}

# Current settings (A4988 potentiometer adjustment)
MOTOR_CURRENT = 0.8A   # Adjust based on motor specifications
```

### Coordinate System
```
Origin: Base joint center
X-axis: Forward (positive extends arm)
Y-axis: Left (positive rotates counterclockwise)
Z-axis: Up (currently unused in 2-DOF system)

Joint 0: Base rotation (around Z-axis)
Joint 1: End effector angle (relative to base segment)
```

---

**This hardware configuration enables precise, unlimited rotation control with high reliability and safety.** 