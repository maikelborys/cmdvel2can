# Plan History and Development Workflow

## **Plan for Package Creation and Development Workflow**

### **1. ROS2 Package Creation**
We will create a new ROS2 package for publishing CAN messages based on `cmd_vel`. This package will include a `plan.md` file to document the development process, maintain context, and ensure consistency.

---

### **2. Package Structure**
The package will have the following structure:
```
cmdvel_to_can_bridge/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── can_interface.hpp
│   ├── velocity_converter.hpp
│   ├── can_message_builder.hpp
│   ├── cmdvel_to_can_node.hpp
│   ├── safety_system.hpp
├── src/
│   ├── can_interface.cpp
│   ├── velocity_converter.cpp
│   ├── can_message_builder.cpp
│   ├── cmdvel_to_can_node.cpp
│   ├── safety_system.cpp
├── config/
│   └── params.yaml
├── launch/
│   └── cmdvel_to_can.launch.py
├── docs/
│   └── plan.md  # Development plan and context
```

---

### **3. `plan.md` File**
The `plan.md` file will serve as a living document to track the development process, decisions, and context. It will include:
1. **Overview**: High-level description of the package and its purpose.
2. **Development Goals**: Current objectives and milestones.
3. **Design Decisions**: Key decisions made during development.
4. **Challenges and Solutions**: Issues encountered and how they were resolved.
5. **Future Plans**: Features or improvements to be implemented later.

---

### **4. Modular Development Approach**
You are currently following a modular development approach, which is a good choice for the following reasons:
1. **Separation of Concerns**: Each package has a clear responsibility (e.g., odometry, CAN publishing).
2. **Ease of Debugging**: Issues can be isolated to specific modules.
3. **Scalability**: New features can be added without affecting existing functionality.
4. **Integration Flexibility**: Modular packages can be integrated with ROS2 Control or other systems in the future.

---

### **5. Future Integration with ROS2 Control**
Your idea of integrating the CAN publishing package with ROS2 Control in the future is feasible. Here are two possible approaches:

1. **Direct Integration**:
   - Extend the `diff_vesc_can_ros2_control` package to include CAN publishing functionality.
   - Use ROS2 Control's hardware interface to manage CAN communication.
   - Pros:
     - Unified system for simulation and real hardware.
     - Leverages ROS2 Control's existing infrastructure.
   - Cons:
     - Higher complexity during development.
     - Requires deep understanding of ROS2 Control.

2. **Modular Integration**:
   - Keep the CAN publishing package separate.
   - Use ROS2 Control for high-level control and simulation.
   - Use the CAN publishing package for real hardware communication.
   - Pros:
     - Easier to debug and maintain.
     - Allows independent development and testing of modules.
   - Cons:
     - Requires additional effort to bridge ROS2 Control and the CAN publishing package.

**Recommendation**: Stick with the modular approach for now. Once the CAN publishing package is stable, you can explore integrating it with ROS2 Control.

---

### **6. Next Steps**
1. Create the `cmdvel_to_can_bridge` package with the structure outlined above.
2. Add a `plan.md` file to document the development process.
3. Implement the basic functionality of the package (e.g., CAN communication, velocity conversion).
4. Test the package on the real robot (with wheels off the ground).
5. Update the `plan.md` file with progress and decisions.

---

---

## **ENHANCED SAFETY SYSTEM INTEGRATION**

### **Plan for Integrating Safety and Incremental Velocity Adjustment System**

To ensure the safety of the motor controllers and the robot, we will integrate a comprehensive security system that validates and adjusts commands before sending them to the motors. This system will rely on the `cmd_vel_real` topic to monitor the actual velocity of the robot and incrementally adjust the commands sent to the motors.

#### **Enhanced Objectives**
1. **Pre-Launch Validation**:
   - Ensure that the `cmd_vel_real` topic is publishing messages before starting the node.
   - Prevent the node from running if no real odometry data is available.

2. **Incremental Velocity Adjustment**:
   - Compare the desired velocity (`cmd_vel`) with the actual velocity (`cmd_vel_real`).
   - Gradually adjust the duty cycle to avoid abrupt changes that could damage the motors or controllers.
   - Implement a configurable curve for smooth acceleration and deceleration.

3. **Safety Mechanisms**:
   - Prevent sending commands that would cause rapid direction changes (e.g., forward to backward within milliseconds).
   - Log warnings or errors if unsafe conditions are detected.
   - Monitor motor temperature and current draw (if available via CAN)

#### **Enhanced Package Structure**
```
cmdvel_to_can_bridge_ros2/
├── include/cmdvel_to_can_bridge_ros2/
│   ├── can_interface.hpp
│   ├── velocity_converter.hpp
│   ├── can_message_builder.hpp
│   ├── cmdvel_to_can_node.hpp
│   ├── safety_system.hpp          # Enhanced: Safety and incremental adjustment logic
│   ├── velocity_monitor.hpp       # New: Real-time velocity monitoring
│   └── incremental_controller.hpp # New: Smooth velocity transitions
├── src/
│   ├── can_interface.cpp
│   ├── velocity_converter.cpp
│   ├── can_message_builder.cpp
│   ├── cmdvel_to_can_node.cpp
│   ├── safety_system.cpp          # Enhanced: Comprehensive safety implementation
│   ├── velocity_monitor.cpp       # New: cmd_vel_real monitoring
│   └── incremental_controller.cpp # New: Incremental adjustment logic
```

#### **Implementation Details**

##### **Pre-Launch Validation**
- **Purpose**: Ensure that the `cmd_vel_real` topic is active before starting the node.
- **Implementation**:
  - Subscribe to `cmd_vel_real` topic during node initialization
  - Wait for configurable timeout (default: 5 seconds) to receive messages
  - Validate message frequency and content quality
  - Block node operation if validation fails

##### **Incremental Velocity Adjustment**
- **Purpose**: Gradually adjust the duty cycle to match the desired velocity.
- **Features**:
  - Configurable acceleration/deceleration curves (linear, exponential, S-curve)
  - Maximum delta velocity per cycle to prevent abrupt changes
  - Velocity error threshold for stability detection
  - Adaptive adjustment based on system response

##### **Enhanced Safety Mechanisms**
- **Direction Change Protection**: Prevent rapid forward/backward transitions
- **Velocity Limit Enforcement**: Hardware-specific maximum velocities
- **Emergency Stop Integration**: Immediate halt capability
- **Motor Protection**: Current and temperature monitoring
- **Communication Health**: CAN bus integrity monitoring

#### **Configuration Parameters**
```yaml
safety_system:
  # Pre-launch validation
  cmd_vel_real_timeout: 5.0          # seconds
  min_message_frequency: 5.0         # Hz
  
  # Incremental adjustment
  max_velocity_delta: 0.1            # m/s per cycle
  acceleration_curve: "s_curve"      # linear, exponential, s_curve
  velocity_tolerance: 0.02           # m/s
  adjustment_frequency: 50.0         # Hz
  
  # Safety thresholds
  max_direction_change_rate: 0.5     # m/s per second
  emergency_stop_deceleration: 2.0   # m/s²
  max_motor_current: 10.0            # amperes
  max_motor_temperature: 80.0        # celsius
  
  # Communication monitoring
  can_timeout: 100                   # milliseconds
  watchdog_timeout: 500              # milliseconds
```

### **Changelog**
#### **July 23, 2025**
- Created the `cmdvel_to_can_bridge_ros2` package.
- Added `plan.md` to document the development process.
- Implemented the initial structure and basic functionality.
- Decided to follow a modular development approach.
- **ENHANCED**: Integrated comprehensive safety system with incremental velocity adjustment.
- **NEW**: Added pre-launch validation for `cmd_vel_real` topic.
- **NEW**: Implemented smooth velocity transition algorithms.
- **NEW**: Added motor protection and communication health monitoring.
- Recommended future integration with ROS2 Control.
