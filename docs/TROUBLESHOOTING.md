# Troubleshooting Guide

## Common Issues

### 1. CAN Interface Not Available
**Symptoms**: 
- Error: "Failed to open CAN socket"
- Node crashes on startup

**Solutions**:
```bash
# Check if CAN interface exists
ip link show can0

# If not available, set up CAN interface
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# For real hardware (adjust bitrate as needed)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# For testing with virtual CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. No CMD_VEL Messages Received
**Symptoms**:
- Node starts but no CAN messages sent
- No response to teleop commands

**Debug Steps**:
```bash
# Check if cmd_vel topic exists
ros2 topic list | grep cmd_vel

# Monitor cmd_vel messages
ros2 topic echo /cmd_vel

# Check topic connections
ros2 node info /cmdvel_to_can_bridge

# Test with manual command
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

### 4. VESC Not Responding to Duty Cycle Commands
**Symptoms**:
- CAN messages sent but motors don't move
- Expected duty cycle values not producing motion

**Verified Solutions**:
```bash
# Test with known working commands (from hardware testing)
# Left VESC (ID 28) - 5% duty cycle
cansend can0 0000001C#00.00.13.88

# Right VESC (ID 46) - 5% duty cycle  
cansend can0 0000002E#00.00.13.88

# Stop both (confirmed working)
cansend can0 0000001C#00.00.00.00
cansend can0 0000002E#00.00.00.00

# Check message format:
# - CAN ID: 0x0000001C (left) or 0x0000002E (right)
# - DLC: 4 bytes
# - Data: 32-bit signed integer, big-endian
# - Range: -100000 to +100000 for -100% to +100%
```

**Hardware Verification**:
- VESC IDs: 28 (0x1C) for left, 46 (0x2E) for right
- Duty cycle scale: ±100000 = ±100%
- Message format verified working on real hardware

### 4. Emergency Stop Triggered
**Symptoms**:
- Robot stops unexpectedly
- Safety warnings in logs

**Resolution**:
```bash
# Check safety system status
ros2 service call /reset_safety std_srvs/srv/Trigger

# Monitor diagnostics
ros2 topic echo /diagnostics

# Adjust watchdog timeout if needed
ros2 param set /cmdvel_to_can_bridge watchdog_timeout 1.0
```

### 5. High Latency/Jitter
**Symptoms**:
- Jerky robot movement
- Inconsistent response to commands

**Optimization**:
```bash
# Check system load
htop

# Monitor CAN bus utilization
canbusload can0@500000

# Adjust command frequency
ros2 param set /cmdvel_to_can_bridge command_frequency 100.0

# Check for RT kernel if needed
uname -r | grep rt
```

## Debug Mode

### Enable Debug Logging
```bash
# Set debug level
ros2 log level set cmdvel_to_can_bridge debug

# Launch in debug mode
ros2 launch cmdvel_to_can_bridge_ros2 debug.launch.py
```

### Debug Launch File Features
- Detailed logging enabled
- CAN traffic monitoring
- Parameter validation
- Performance metrics
- Safety system status

## Performance Monitoring

### Key Metrics
- Command latency: < 10ms
- CAN bus utilization: < 50%
- CPU usage: < 5%
- Memory usage: < 100MB

### Monitoring Commands
```bash
# Check node performance
ros2 node info /cmdvel_to_can_bridge --verbose

# Monitor resource usage
top -p $(pgrep -f cmdvel_to_can_bridge)

# CAN statistics
cat /proc/net/can/stats
```

## Safety System Debug

### Watchdog Issues
```bash
# Check last command timestamp
ros2 topic echo /diagnostics --field data[0].values[0].value

# Test watchdog manually
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' --once
# Wait for timeout...
```

### Velocity Validation
```bash
# Test velocity limits
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 5.0}}' --once
# Should be clamped to max_velocity

# Check validation status
ros2 param get /cmdvel_to_can_bridge max_velocity
```

## Hardware Debugging

### CAN Bus Health
```bash
# Check error frames
candump can0 -e

# Monitor bus load
canbusload can0@500000

# Test loopback
cansend can0 123#DEADBEEF
candump can0 -n 1
```

### VESC Diagnostics
```bash
# Monitor VESC status messages
candump can0 | grep "1B[12][A-F0-9]"

# Check for fault codes
# Look for STATUS_4 messages with fault information
```

## Log Analysis

### Important Log Patterns
```
[ERROR] Failed to send CAN frame: Device busy
→ CAN bus overloaded or interface down

[WARN] Velocity command timeout
→ No cmd_vel received within watchdog period

[ERROR] VESC ID validation failed
→ Received message from unexpected VESC ID
```

### Log Filtering
```bash
# Filter CAN-related logs
ros2 log | grep -i can

# Filter safety system logs
ros2 log | grep -i safety

# Filter performance logs
ros2 log | grep -i latency
```

## Recovery Procedures

### Soft Reset
```bash
# Reset safety system
ros2 service call /reset_safety std_srvs/srv/Trigger

# Restart node
ros2 lifecycle set /cmdvel_to_can_bridge shutdown
ros2 lifecycle set /cmdvel_to_can_bridge configure
```

### Hard Reset
```bash
# Kill and restart node
pkill -f cmdvel_to_can_bridge
ros2 launch cmdvel_to_can_bridge_ros2 cmdvel_to_can.launch.py

# Reset CAN interface
sudo ip link set down can0
sudo ip link set up can0
```

## Contact Information
For additional support, refer to:
- Package documentation: `docs/README.md`
- Development history: `docs/plan_history.md`
- API reference: `docs/API.md`
