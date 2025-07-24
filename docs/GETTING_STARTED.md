
# Getting Started and Debugging Guide

## 🚀 Quick Start

### 1. Build and Setup
```bash
# Build the package
cd ~/robot_ws
colcon build --packages-select cmdvel2can

# Source the workspace
source install/setup.bash

# Verify the package is available
ros2 pkg list | grep cmdvel2can
```

### 2. Pre-Flight Checks
Before running the nodes, ensure these prerequisites:

```bash
# 1. Check CAN interface is available
ip link show can0

# 2. If can0 doesn't exist, set it up:
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 3. Verify CAN interface is up
ip link show can0
# Should show: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP

# 4. Check if cmd_vel_real topic exists (required for safety)
ros2 topic list | grep cmd_vel_real

# 5. If cmd_vel_real doesn't exist, start the odometry node first (if required):
ros2 run diff_vesc_can_ros2_pkg_cpp vesc_odometry_node
```

### 3. Manual Launch (Modular Bridge)
```bash
# Terminal 1: Start CAN bridge node
ros2 run cmdvel2can can_bridge_node --ros-args -p can_interface:=can0

# Terminal 2: Start control node
ros2 run cmdvel2can cmdvel2can_node

# Terminal 3: Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

### 4. Simulation
- For simulation, run a simulation bridge node that subscribes to `can_tx` and updates the simulated robot (e.g., in Gazebo or IsaacSim).
- The control node does not need to change for simulation vs. hardware.

---

## 🔧 Development Workflow

### Phase 1: Basic Node Testing (Start Here!)
```bash
# 1. Test node startup (will fail safely without cmd_vel_real)
ros2 run cmdvel_to_can_bridge_ros2 cmdvel_to_can_bridge_node

# 2. Check node status
ros2 node list | grep cmdvel_to_can_bridge

# 3. Examine topics
ros2 topic list | grep -E "(cmd_vel|diagnostics)"

# 4. Check parameters
ros2 param list /cmdvel_to_can_bridge
```

### Phase 2: CAN Interface Testing
```bash
# 1. Monitor CAN traffic in separate terminal
candump can0

# 2. Test manual VESC commands (WHEELS OFF GROUND!)

## Odometry and Ticks (2025 empirical calibration)
- 3 Hall sensors = 6 electrical states per mechanical revolution
- 23 poles (mechanical pulses per revolution)
- 138 ticks per mechanical revolution (23 × 6)
- Wheel diameter: 0.3556m
- Wheel circumference: π × 0.3556m = 1.117m
- Distance per tick: 1.117m / 138 ≈ 0.008094m (8.1 mm per tick)
- VESC STATUS_5 messages report electrical revolutions; divide by 6 for mechanical.
- Use 138 as the correct number of ticks per wheel turn for odometry and calibration.
- To convert duty cycle to distance/sec: (ticks/sec per duty) × 8.1 mm/tick (see calibration data).
# Stop command (safe to start with)
cansend can0 0000001C#00.00.00.00  # Left VESC stop
cansend can0 0000002E#00.00.00.00  # Right VESC stop

# Small test movement (2% duty cycle)
cansend can0 0000001C#00.00.07.D0  # Left VESC 2%
sleep 2
cansend can0 0000001C#00.00.00.00  # Stop

# 3. Verify CAN messages are being sent
# You should see the messages in candump output
```

### Phase 3: ROS2 Integration Testing
```bash
# 1. Start the full system (ensure cmd_vel_real is running first)
ros2 launch cmdvel_to_can_bridge_ros2 debug.launch.py

# 2. Test with manual cmd_vel commands
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 3. Monitor diagnostics
ros2 topic echo /diagnostics

# 4. Check safety system
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

## 🐛 Debugging Strategy

### Level 1: Build and Node Issues
```bash
# Clean build if needed
rm -rf build/cmdvel_to_can_bridge_ros2 install/cmdvel_to_can_bridge_ros2
colcon build --packages-select cmdvel_to_can_bridge_ros2

# Debug build with symbols
colcon build --packages-select cmdvel_to_can_bridge_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Check for compilation warnings
colcon build --packages-select cmdvel_to_can_bridge_ros2 --event-handlers console_direct+
```

### Level 2: Runtime Debugging
```bash
# Enable debug logging
ros2 log level set cmdvel_to_can_bridge debug

# Run with verbose output
ros2 run cmdvel_to_can_bridge_ros2 cmdvel_to_can_bridge_node --ros-args --log-level debug

# Check for error messages
ros2 log
```

### Level 3: CAN Communication Debugging
```bash
# Monitor all CAN traffic
candump can0 -t z -x

# Monitor only VESC responses
candump can0 | grep -E "(1C|2E)"

# Check CAN interface statistics
cat /proc/net/can/stats

# Test CAN interface with loopback
cansend can0 123#DEADBEEF
```

### Level 4: Safety System Debugging
```bash
# Check safety parameters
ros2 param get /cmdvel_to_can_bridge safety_system.cmd_vel_real_timeout

# Test safety triggers
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'  # Should be clamped

# Monitor safety events
ros2 topic echo /diagnostics | grep -i safety
```

## 🛠️ Implementation Priority

### Start Here (Immediate Development):
1. **CAN Interface Implementation** (`src/can_interface.cpp`)
   - SocketCAN socket creation and binding
   - Basic send/receive functionality
   - Error handling

2. **Basic Node Structure** (`src/cmdvel_to_can_node.cpp`)
   - ROS2 node initialization
   - Parameter loading
   - Topic subscription setup

3. **Message Builder** (`src/can_message_builder.cpp`)
   - Implement `buildDutyCycleCommand()` with verified protocol
   - Test with known working commands

### Next Steps:
4. **Safety System** (`src/safety_system.cpp`)
   - Basic velocity validation
   - Emergency stop functionality

5. **Velocity Converter** (`src/velocity_converter.cpp`)
   - Differential drive kinematics
   - cmd_vel to wheel velocities

6. **Advanced Features**:
   - Velocity monitoring
   - Incremental controller
   - Enhanced diagnostics

## 🔍 Testing Commands

### Safe Testing Sequence
```bash
# 1. ALWAYS start with wheels off the ground!

# 2. Test stop commands first
cansend can0 0000001C#00.00.00.00
cansend can0 0000002E#00.00.00.00

# 3. Test very small movements (1% duty cycle)
cansend can0 0000001C#00.00.03.E8  # 1% left wheel
sleep 1
cansend can0 0000001C#00.00.00.00  # stop

# 4. Test right wheel
cansend can0 0000002E#00.00.03.E8  # 1% right wheel  
sleep 1
cansend can0 0000002E#00.00.00.00  # stop

# 5. Gradually increase duty cycle for testing
# 2%: #00.00.07.D0
# 3%: #00.00.0B.B8
# 5%: #00.00.13.88
```

### Emergency Procedures
```bash
# IMMEDIATE STOP (use in emergency)
cansend can0 0000001C#00.00.00.00 && cansend can0 0000002E#00.00.00.00

# Or via ROS2
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Kill all nodes if needed
pkill -f cmdvel_to_can_bridge
```

## 📊 Monitoring and Validation

### Real-time Monitoring Setup
```bash
# Terminal 1: CAN traffic
candump can0 -t z

# Terminal 2: ROS2 topics  
ros2 topic echo /cmd_vel

# Terminal 3: Diagnostics
ros2 topic echo /diagnostics

# Terminal 4: Node logs
ros2 log
```

### Key Metrics to Watch
- **CAN Message Rate**: Should match command_frequency (50Hz)
- **Duty Cycle Values**: Should be within configured limits (±10%)
- **Safety Triggers**: Monitor for emergency stops or warnings
- **cmd_vel_real Feedback**: Ensure velocity feedback is available

## 🔄 Iterative Development Approach

### Iteration 1: Minimal Viable Product
- [x] Package builds successfully
- [ ] Basic CAN interface (send only)
- [ ] Simple duty cycle commands
- [ ] Manual testing with cansend verification

### Iteration 2: Basic ROS2 Integration
- [ ] cmd_vel subscription
- [ ] Basic velocity to duty cycle conversion
- [ ] Safety limits implementation

### Iteration 3: Enhanced Safety
- [ ] cmd_vel_real monitoring
- [ ] Emergency stop functionality
- [ ] Incremental velocity adjustments

### Iteration 4: Production Ready
- [ ] Full diagnostics
- [ ] Performance optimization
- [ ] Documentation completion

## 📝 Development Log Template

Keep track of your progress:
```
Date: YYYY-MM-DD
Iteration: X
Goal: [What you're trying to implement]
Status: [Working/Broken/Partial]
Issues: [Any problems encountered]
Next Steps: [What to do next]
Test Results: [What worked/didn't work]
```

Start with **Iteration 1** and focus on getting basic CAN communication working first!
