## Velocity to Duty Cycle Formula (2025 empirical calibration)

To convert a desired wheel velocity (in m/s) to duty cycle:

    duty_cycle = velocity_mps / 7.857

Where:
- velocity_mps: desired wheel velocity in meters per second
- 7.857: empirical constant (0.0081 m/tick × 970 ticks/sec/duty)

**Use this formula for all cmdvel-to-duty conversions in your code.**
# ROS2 VESC CAN Bridge - API Reference

## ROS2 Interface

### Topics

#### Subscriptions
- **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
  - Robot velocity commands
  - **linear.x**: Forward/backward velocity (m/s)
  - **angular.z**: Rotational velocity (rad/s)
  - **Limits**: 
    - Linear: ±2.0 m/s
    - Angular: ±3.0 rad/s

### Parameters

#### Robot Configuration
```yaml
wheel_separation: 0.370      # Distance between wheels (m)
wheel_diameter: 0.356        # Wheel diameter (m)
```

#### VESC Configuration  
```yaml
left_vesc_id: 28            # Left motor VESC ID
right_vesc_id: 46           # Right motor VESC ID
can_interface: "can0"       # CAN interface name
```

#### Control Parameters
```yaml
max_velocity: 2.0           # Maximum linear velocity (m/s)
max_angular_velocity: 3.0   # Maximum angular velocity (rad/s)
command_frequency: 20.0     # Control loop frequency (Hz)
watchdog_timeout: 0.5       # Safety timeout (seconds)
```

### Launch Files

#### Basic Launch
```xml
<!-- launch/cmdvel_to_can_bridge.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmdvel_to_can_bridge_ros2',
            executable='cmdvel_to_can_bridge_node',
            name='cmdvel_to_can_bridge',
            parameters=[{
                'wheel_separation': 0.370,
                'wheel_diameter': 0.356,
                'left_vesc_id': 28,
                'right_vesc_id': 46,
                'can_interface': 'can0',
                'max_velocity': 2.0,
                'max_angular_velocity': 3.0
            }]
        )
    ])
```

## CAN Protocol Reference


### VESC Duty Cycle, Ticks, and Odometry (2025 empirical calibration)
- **3 Hall sensors** = 6 electrical states per mechanical revolution
- **23 poles** (mechanical pulses per revolution)
- **138 ticks per mechanical revolution** (23 × 6)
- **Wheel diameter**: 0.3556m
- **Wheel circumference**: π × 0.3556m = 1.117m
- **Distance per tick**: 1.117m / 138 ≈ 0.008094m (8.1 mm per tick)
- VESC STATUS_5 messages report electrical revolutions; divide by 6 for mechanical.
- Use 138 as the correct number of ticks per wheel turn for odometry and calibration.
- To convert duty cycle to distance/sec: (ticks/sec per duty) × 8.1 mm/tick (see calibration data).

#### Frame Format
- **CAN ID**: `0x8000001C` (left), `0x8000002E` (right) 
- **DLC**: 4 bytes
- **Data**: 32-bit signed integer, big-endian
- **Scaling**: `duty_percentage * 100000`

#### Example Commands
```
# 5% forward duty cycle
CAN ID: 0x8000001C
Data: 0x00 0x00 0x04 0xE2  (1250 decimal)

# Stop (0% duty cycle)  
CAN ID: 0x8000001C
Data: 0x00 0x00 0x00 0x00  (0 decimal)

# 5% reverse duty cycle
CAN ID: 0x8000001C  
Data: 0xFF 0xFF 0xFB 0x1E  (-1250 decimal)
```

### Manual CAN Commands
```bash
# Forward movement (5% duty to left VESC)
cansend can0 8000001C#00.00.04.E2

# Stop command
cansend can0 8000001C#00.00.00.00

# Monitor VESC responses
candump can0 | grep -E "(001C|002E|1B1C|1B2E)"
```

## C++ API Reference

### Core Classes

#### CmdVelToCANNode
```cpp
class CmdVelToCANNode : public rclcpp::Node {
public:
    CmdVelToCANNode();
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void controlLoop();
    bool sendCANCommands(double left_duty, double right_duty);
    
    // ROS components
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Hardware interfaces
    std::unique_ptr<CANInterface> can_interface_;
    std::unique_ptr<CANMessageBuilder> message_builder_;
};
```

#### CANInterface
```cpp
class CANInterface {
public:
    bool initialize(const std::string& interface_name);
    bool sendFrame(const can_frame& frame);
    bool receiveFrame(can_frame& frame);
    void shutdown();
    
private:
    int socket_fd_;
    bool is_initialized_;
    std::string interface_name_;
};
```

#### CANMessageBuilder  
```cpp
class CANMessageBuilder {
public:
    CANMessageBuilder(uint8_t left_vesc_id, uint8_t right_vesc_id);
    
    can_frame buildDutyCycleCommand(uint8_t vesc_id, double duty_cycle);
    
private:
    uint8_t left_vesc_id_;
    uint8_t right_vesc_id_;
};
```

### Key Functions

#### Kinematics Conversion
```cpp
// Convert cmd_vel to wheel velocities
std::pair<double, double> cmdVelToWheelVelocities(
    double linear_vel, double angular_vel, 
    double wheel_separation) {
    
    double left_vel = linear_vel - (angular_vel * wheel_separation / 2.0);
    double right_vel = linear_vel + (angular_vel * wheel_separation / 2.0);
    
    return std::make_pair(left_vel, right_vel);
}

// Convert velocity to duty cycle  
double velocityToDutyCycle(double velocity, double max_velocity) {
    return std::clamp(velocity / max_velocity, -1.0, 1.0);
}
```

#### CAN Frame Building
```cpp
can_frame buildDutyCycleCommand(uint8_t vesc_id, double duty_cycle) {
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    // CRITICAL: Extended frame format required for VESC
    frame.can_id = static_cast<uint32_t>(vesc_id) | CAN_EFF_FLAG;
    frame.can_dlc = 4;
    
    // Scale and encode duty cycle
    int32_t scaled_duty = static_cast<int32_t>(duty_cycle * 100000);
    frame.data[0] = (scaled_duty >> 24) & 0xFF;
    frame.data[1] = (scaled_duty >> 16) & 0xFF;
    frame.data[2] = (scaled_duty >> 8) & 0xFF;
    frame.data[3] = scaled_duty & 0xFF;
    
    return frame;
}
```

## Error Codes & Diagnostics

### Common Log Messages

#### Success Messages
```
[INFO] [can_interface]: CAN interface can0 initialized successfully
[INFO] [cmdvel_to_can_bridge]: ✓ Sent left VESC command: duty=X.XXX
[INFO] [can_interface]: Sent CAN frame: ID=0x8000001C, DLC=4
```

#### Error Messages
```
[ERROR] [can_interface]: Failed to initialize CAN socket: Permission denied
[ERROR] [can_interface]: Failed to send CAN frame: Network is down
[WARN] [cmdvel_to_can_bridge]: Velocity command exceeds limits
```

### Diagnostic Commands
```bash
# Check node status
ros2 node list | grep cmdvel_to_can_bridge

# Monitor node output
ros2 run cmdvel_to_can_bridge_ros2 cmdvel_to_can_bridge_node --ros-args --log-level debug

# Check topic connections
ros2 topic info /cmd_vel

# Test with minimal command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.01}" --once
```

---
*Last Updated: July 23, 2025*
