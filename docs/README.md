## Data Flow: How /cmd_vel is Processed (Detailed)

This section describes, step by step, how a `/cmd_vel` message is processed by the system, referencing the main classes and functions in the code.

### 1. ROS2 Topic Reception
- The control node (`Cmdvel2CanNode`) subscribes to `/cmd_vel` (type: `geometry_msgs/msg/Twist`).
- When a message arrives, the callback stores it in `last_cmd_vel_` and updates the timestamp `last_cmdvel_time_`.

### 2. Safety Check
- The node also subscribes to `/cmd_vel_real` for safety. If this topic is not alive (checked by timestamp), the node will not send CAN commands.

### 3. Control Loop (Timer)
- A timer runs at `command_frequency` (e.g., 50 Hz).
- On each tick, the `control_loop()` function is called.
- Inside `control_loop()`:
  - Checks if `/cmd_vel_real` is alive (fresh message within `watchdog_timeout`).
  - If not, logs a warning and skips sending CAN.

### 4. Kinematics Conversion
- The last received `geometry_msgs/msg/Twist` is converted to left/right wheel velocities using differential drive kinematics:
  - `left = linear.x - (angular.z * wheel_separation / 2.0)`
  - `right = linear.x + (angular.z * wheel_separation / 2.0)`
- Velocities are clamped to `[-max_velocity, max_velocity]`.

### 5. Duty Cycle Mapping
- Each wheel velocity is mapped to a duty cycle:
  - `duty = velocity / max_velocity` (clamped to [-1.0, 1.0])
  - Scaled to VESC format: `int32_t scaled = duty * 100000`

### 6. CAN Frame Construction
- The function `build_duty_cycle_frame(vesc_id, duty)` packs the duty cycle as a 4-byte big-endian integer in a `can_msgs::msg::Frame`.
- The CAN ID is set to `(vesc_id | CAN_EFF_FLAG)` for extended frame format.

### 7. CAN Frame Publishing
- The node publishes the CAN frame to the `can_tx` topic using `can_pub_->publish(...)`.
- This happens for both left and right VESCs.

### 8. CAN Bridge Node
- The separate `can_bridge_node` subscribes to `can_tx` and writes the frame to the real CAN bus using SocketCAN.

### 9. VESC Receives Command
- The VESC receives the CAN frame and applies the duty cycle to the motor.

---

**Key Classes/Functions:**
- `Cmdvel2CanNode`: Main ROS2 node class
- `control_loop()`: Main timer-driven processing function
- `cmdvel_to_wheel_velocities()`: Kinematics conversion
- `velocity_to_duty_cycle()`: Duty cycle mapping
- `build_duty_cycle_frame()`: CAN frame construction

**Performance Notes:**
- The timer frequency (`command_frequency`) controls how often commands are sent (e.g., 50 Hz).
- For minimal delay, you can also send CAN frames immediately in the `/cmd_vel` callback.

---
# ROS2 VESC CAN Bridge - Complete Documentation

## 🎉 Project Status: WORKING! ✅

This project provides a complete ROS2 interface to control VESC motor controllers via CAN bus using duty cycle commands. The system successfully converts ROS2 `geometry_msgs/Twist` messages to VESC CAN commands for differential drive robots.

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Software Architecture](#software-architecture)
4. [Installation & Setup](#installation--setup)
5. [Usage Guide](#usage-guide)
6. [Technical Details](#technical-details)
7. [Troubleshooting](#troubleshooting)
8. [Development History](#development-history)

---

## System Overview

### What This System Does
- **Input**: ROS2 `geometry_msgs/Twist` messages on `/cmd_vel` topic
- **Output**: CAN frames to VESC motor controllers using duty cycle commands
- **Result**: Differential drive robot movement with precise wheel control

### Key Features
- ✅ Real-time cmd_vel to CAN translation
- ✅ Differential drive kinematics
- ✅ Safety limits and watchdog protection  
- ✅ Configurable VESC IDs and parameters
- ✅ Comprehensive logging and debugging
- ✅ Production-ready C++ implementation

---

## Hardware Configuration

### VESC Motor Controllers
- **Left VESC**: ID 28 (0x1C) - CAN ID `0x0000001C`
- **Right VESC**: ID 46 (0x2E) - CAN ID `0x0000002E`
- **CAN Interface**: `can0` 
- **CAN Protocol**: Extended frame format with duty cycle commands

### Robot Physical Parameters
- **Wheel Separation**: 0.370 m (distance between wheel centers)
- **Wheel Diameter**: 0.356 m
- **Max Linear Velocity**: 2.00 m/s
- **Max Angular Velocity**: 3.00 rad/s

### CAN Bus Setup
```bash
# CAN interface should be configured and active
ip link show can0  # Should show UP state
candump can0       # Should show VESC telemetry traffic
```

---


## System Overview

### What This System Does
- **Input**: ROS2 `geometry_msgs/Twist` messages on `/cmd_vel` topic
- **Output**: CAN frames to VESC motor controllers using duty cycle commands
- **Result**: Differential drive robot movement with precise wheel control

### Key Features
- ✅ Real-time cmd_vel to CAN translation
- ✅ Differential drive kinematics
- ✅ Safety limits and watchdog protection
- ✅ Configurable VESC IDs and parameters
- ✅ Comprehensive logging and debugging
- ✅ Production-ready C++ implementation
- ✅ **Modular bridge architecture**: hardware-agnostic control node + bridge node
- ✅ **Simulation-ready**: swap bridge node for simulation node

---

## Modular Software Architecture

### Node Structure
- **Control Node** (`cmdvel2can_node`):
# Install CAN utilities
sudo apt install can-utils

# Ensure ROS2 Humble is installed and sourced
- **CAN Bridge Node** (`can_bridge_node`):
source /opt/ros/humble/setup.bash
```

### Build the Package

### Launching (Manual)
1. **Source your workspace:**
   ```bash
   source ~/robot_ws/install/setup.bash
   ```
2. **Start the CAN bridge node:**
   ```bash
   ros2 run cmdvel2can can_bridge_node --ros-args -p can_interface:=can0
   ```
3. **In a new terminal, start the control node:**
   ```bash
   source ~/robot_ws/install/setup.bash
   ros2 run cmdvel2can cmdvel2can_node
   ```
4. **Send velocity commands:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
   ```

---

## Hardware Configuration

### VESC Motor Controllers
- **Left VESC**: ID 28 (0x1C) - CAN ID `0x0000001C`
- **Right VESC**: ID 46 (0x2E) - CAN ID `0x0000002E`
- **CAN Interface**: `can0`
- **CAN Protocol**: Extended frame format with duty cycle commands

### Robot Physical Parameters
- **Wheel Separation**: 0.370 m (distance between wheel centers)
- **Wheel Diameter**: 0.356 m
- **Max Linear Velocity**: 2.00 m/s
- **Max Angular Velocity**: 3.00 rad/s

### CAN Bus Setup
```bash
# CAN interface should be configured and active
ip link show can0  # Should show UP state
candump can0       # Should show VESC telemetry traffic
```

---

## Simulation & Extensibility

- For simulation, run a simulation bridge node that subscribes to `can_tx` and updates the simulated robot (e.g., in Gazebo or IsaacSim).
- The control node does not need to change for simulation vs. hardware.
- This modularity enables easy testing, debugging, and future hardware upgrades.

---

## Troubleshooting

- If wheels do not move, check CAN traffic with `candump can0`.
- Ensure both nodes are running and `can_tx` is being published.
- For simulation, verify the simulation bridge node is active.

---

## Development History
```bash
cd /home/robot/robot_ws
colcon build --packages-select cmdvel_to_can_bridge_ros2
source install/setup.bash
```

### Verify CAN Interface
```bash
# Check CAN interface status
ip link show can0

# Monitor VESC telemetry (should show regular traffic)
candump can0 | head -10
```

---

## Usage Guide

### Starting the Bridge Node
```bash
# Method 1: Direct executable
cd /home/robot/robot_ws
source install/setup.bash
./install/cmdvel_to_can_bridge_ros2/lib/cmdvel_to_can_bridge_ros2/cmdvel_to_can_bridge_node

# Method 2: Using ros2 run (alternative)
ros2 run cmdvel_to_can_bridge_ros2 cmdvel_to_can_bridge_node
```

### Expected Node Output
```
[INFO] [cmdvel_to_can_bridge]: Initializing CmdVel to CAN Bridge Node
[INFO] [cmdvel_to_can_bridge]: Parameters loaded:
[INFO] [cmdvel_to_can_bridge]:   - Wheel separation: 0.370 m
[INFO] [cmdvel_to_can_bridge]:   - Wheel diameter: 0.356 m
[INFO] [cmdvel_to_can_bridge]:   - Left VESC ID: 28
[INFO] [cmdvel_to_can_bridge]:   - Right VESC ID: 46
[INFO] [cmdvel_to_can_bridge]:   - CAN interface: can0
[INFO] [can_interface]: CAN interface can0 initialized successfully
[INFO] [cmdvel_to_can_bridge]: CmdVel to CAN Bridge Node initialized successfully
```

### Sending Movement Commands

#### Basic Forward Movement
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.1, y: 0.0, z: 0.0}
   angular: {z: 0.0}" --once
```

#### Rotation
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {z: 0.5}" --once
```

#### Stop Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {z: 0.0}" --once
```

### Expected CAN Traffic
When commands are sent, you should see CAN frames like:
```bash
candump can0 | grep -E "(001C|002E)"
# Output: can0  0000001C  [4]  XX XX XX XX  (Left VESC)
#         can0  0000002E  [4]  XX XX XX XX  (Right VESC)
```

---

## Technical Details

### VESC CAN Protocol

#### Duty Cycle Command Format
- **CAN ID**: VESC ID (28 for left, 46 for right)
- **Frame Type**: Extended format (`CAN_EFF_FLAG`)
- **DLC**: 4 bytes
- **Data**: 32-bit signed integer, big-endian
- **Scaling**: `duty_percentage * 100000`

#### Example Commands
```
Forward 5%:  0x00 0x00 0x04 0xE2  (1250 decimal)
Stop 0%:     0x00 0x00 0x00 0x00  (0 decimal)
Reverse -5%: 0xFF 0xFF 0xFB 0x1E  (-1250 decimal)
```

### Kinematics Calculations

#### Differential Drive Equations
```cpp
// Convert cmd_vel to wheel velocities
double wheel_base = 0.370;  // meters
double wheel_radius = 0.356 / 2.0;  // meters

double left_velocity = linear_vel - (angular_vel * wheel_base / 2.0);
double right_velocity = linear_vel + (angular_vel * wheel_base / 2.0);

// Convert to duty cycles (assuming max velocity = 2.0 m/s = 100% duty)
double left_duty = left_velocity / 2.0;   // Normalized to ±1.0
double right_duty = right_velocity / 2.0; // Normalized to ±1.0
```

### Critical Implementation Details

#### CAN Frame Format Fix
**Problem**: Socket programs reported success but didn't transmit to CAN bus.

**Solution**: VESC requires extended CAN frame format:
```cpp
// WRONG (doesn't work):
frame.can_id = static_cast<uint32_t>(vesc_id);

// CORRECT (works):
frame.can_id = static_cast<uint32_t>(vesc_id) | CAN_EFF_FLAG;
```

#### Socket Setup
```cpp
// Proper SocketCAN initialization
int socket_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
struct sockaddr_can addr;
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;  // from SIOCGIFINDEX
bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr));
```

---

## Troubleshooting

### Common Issues

#### 1. "Package not found" Error
```bash
# Solution: Rebuild and source
colcon build --packages-select cmdvel_to_can_bridge_ros2
source install/setup.bash
```

#### 2. No Wheel Movement Despite "Success" Messages
**Symptoms**: Node logs show successful CAN transmission, but wheels don't move.

**Diagnosis**: Check CAN frame format
```bash
# Monitor CAN traffic while sending commands
candump can0 &
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1}" --once

# You should see frames with IDs 0x0000001C and 0x0000002E
# If missing, it's a frame format issue
```

**Solution**: Ensure `CAN_EFF_FLAG` is set in can_message_builder.cpp

#### 3. CAN Interface Down
```bash
# Check interface status
ip link show can0

# If DOWN, configure it:
sudo ip link set can0 up type can bitrate 500000
```

#### 4. Permission Denied
```bash
# Add user to dialout group for CAN access
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Debug Commands

#### Monitor All CAN Traffic
```bash
candump can0
```

#### Test Manual CAN Commands
```bash
# Send test duty cycle command (5% forward to left VESC)
cansend can0 0000001C#00.00.04.E2

# Send stop command
cansend can0 0000001C#00.00.00.00
```

#### Check ROS Topics
```bash
# List active topics
ros2 topic list

# Monitor cmd_vel messages  
ros2 topic echo /cmd_vel

# Send test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1}" --once
```

---

## Development History

### Critical Breakthrough: CAN Frame Format
**Date**: July 23, 2025

**Problem**: Socket programs (including ROS node) reported successful `write()` calls but CAN frames never appeared on the bus.

**Investigation**: Used `strace` to compare working `cansend` vs failing socket programs:
```bash
# Working cansend format:
strace cansend can0 0000001C#00.00.04.E2
# Output: write(3, "\34\0\0\200\4\0\0\0\0\0\4\342\0\0\0\0", 16)

# Failing socket format:  
strace ./test_program
# Output: write(3, "\34\0\0\0\4\0\0\0\0\0\4\342\0\0\0\0", 16)
```

**Key Difference**: Byte 3 was `\200` (0x80) vs `\0` (0x00)

**Root Cause**: Missing `CAN_EFF_FLAG` (extended frame format) in CAN ID.

**Fix**: 
```cpp
// Before (broken):
frame.can_id = static_cast<uint32_t>(vesc_id);

// After (working):
frame.can_id = static_cast<uint32_t>(vesc_id) | CAN_EFF_FLAG;
```

### Project Evolution
1. **Initial Setup**: Basic ROS2 package structure
2. **Protocol Implementation**: VESC CAN duty cycle commands
3. **Integration**: Complete cmd_vel to CAN bridge
4. **Debugging Phase**: Socket transmission issues
5. **Breakthrough**: CAN frame format discovery
6. **Success**: Working robot movement via ROS2

### Key Lessons Learned
- VESC requires extended CAN frame format (`CAN_EFF_FLAG`)
- `write()` success doesn't guarantee CAN bus transmission
- `strace` is invaluable for debugging system call differences
- Manual `cansend` uses different frame format than basic socket programming
- Always verify actual CAN bus traffic with `candump`

---

## Configuration Parameters

### Node Parameters (can be modified in launch files)
```yaml
wheel_separation: 0.370      # meters
wheel_diameter: 0.356        # meters  
left_vesc_id: 28            # CAN ID for left motor
right_vesc_id: 46           # CAN ID for right motor
can_interface: "can0"       # CAN interface name
max_velocity: 2.0           # m/s
max_angular_velocity: 3.0   # rad/s
command_frequency: 20.0     # Hz
watchdog_timeout: 0.5       # seconds
```

### Safety Limits
- Duty cycle clamped to ±100% (-1.0 to +1.0)
- Velocity limits enforced before conversion
- Watchdog stops motors if no commands received

---

## Future Development

### Potential Enhancements
1. **Launch Files**: Create proper ROS2 launch configurations
2. **Parameter Server**: Dynamic parameter reconfiguration  
3. **Odometry**: Add encoder feedback for closed-loop control
4. **Safety Features**: Emergency stop, collision avoidance
5. **Diagnostics**: Health monitoring and error reporting
6. **Testing**: Automated unit and integration tests

### Integration Opportunities
- **Navigation2**: Full autonomous navigation stack
- **ROS2 Control**: Standard robot control interfaces
- **Gazebo Simulation**: Virtual testing environment
- **Teleoperation**: Remote control interfaces

---

## Contact & Support

This documentation covers the complete working ROS2 VESC CAN bridge system. For questions or issues:

1. Check the troubleshooting section above
2. Verify CAN interface and traffic with `candump`
3. Review node logs for error messages
4. Test with manual `cansend` commands to isolate issues

**Remember**: The key to success was discovering that VESC requires extended CAN frame format (`CAN_EFF_FLAG`). Without this, socket programs will report success but won't actually transmit to the CAN bus.

---

*Last Updated: July 23, 2025*  
*Status: PRODUCTION READY ✅*
