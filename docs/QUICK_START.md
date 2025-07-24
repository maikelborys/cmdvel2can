# ROS2 VESC CAN Bridge - Quick Start Guide

## 🚀 Quick Start (5 Minutes)

### 1. Build & Install
```bash
cd /home/robot/robot_ws
colcon build --packages-select cmdvel_to_can_bridge_ros2
source install/setup.bash
```

### 2. Start the Bridge
```bash
./install/cmdvel_to_can_bridge_ros2/lib/cmdvel_to_can_bridge_ros2/cmdvel_to_can_bridge_node
```

### 3. Test Movement
```bash
# Forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0}" --once
```

## ✅ Success Indicators

### Node Startup
```
[INFO] [cmdvel_to_can_bridge]: CmdVel to CAN Bridge Node initialized successfully
[INFO] [can_interface]: CAN interface can0 initialized successfully
```

### Command Execution
```
[INFO] [can_interface]: Sent CAN frame: ID=0x8000001C, DLC=4, Data=00.00.04.E2
[INFO] [cmdvel_to_can_bridge]: ✓ Sent left VESC command: duty=0.013
```

### Physical Movement
- Wheels should move smoothly
- No jerky or erratic behavior
- Immediate response to commands

## 🔧 Quick Troubleshooting

### Problem: No movement despite "success" messages
**Solution**: Check CAN traffic
```bash
candump can0 | grep -E "(001C|002E)"
# Should show frames when sending commands
```

### Problem: "Package not found"
**Solution**: Rebuild and source
```bash
colcon build --packages-select cmdvel_to_can_bridge_ros2
source install/setup.bash
```

### Problem: CAN interface issues
**Solution**: Check interface status
```bash
ip link show can0  # Should show UP
sudo ip link set can0 up type can bitrate 500000  # If needed
```

## 📖 Full Documentation
See `docs/README.md` for complete technical details.

---
*Last Updated: July 23, 2025*
