# cmdvel2can - ROS2 VESC CAN Bridge (Single-File Edition)

## Project Overview

This project provides a **minimal, single-file ROS2 node** to bridge `/cmd_vel` velocity commands to VESC motor controllers via CAN bus. The goal is to maximize clarity, modularity, and safety, while making the code easy to study and extend.

---

## 1. System Architecture

- **Input:** `/cmd_vel` (`geometry_msgs/msg/Twist`)
- **Output:** CAN bus messages (VESC protocol, extended frame)
- **Key Components (all in one .cpp):**
  - CAN socket interface (SocketCAN)
  - Kinematics conversion (Twist → wheel velocities)
  - Duty cycle encoding (for VESC)
  - Safety system: node only active if `/cmd_vel_real` is alive
  - Diagnostics and logging

---

## 2. Parameters & Topics

- **Parameters:**
  - `wheel_separation` (m)
  - `wheel_diameter` (m)
  - **Odometry Calibration (2025):**
    - 3 Hall sensors = 6 electrical states per mechanical revolution
    - 23 poles (mechanical pulses per revolution)
    - 138 ticks per mechanical revolution (23 × 6)
    - Wheel diameter: 0.3556m
    - Wheel circumference: π × 0.3556m = 1.117m
    - Distance per tick: 1.117m / 138 ≈ 0.008094m (8.1 mm per tick)
    - VESC STATUS_5 messages report electrical revolutions; divide by 6 for mechanical.
    - Use 138 as the correct number of ticks per wheel turn for odometry and calibration.
    - To convert duty cycle to distance/sec: (ticks/sec per duty) × 8.1 mm/tick (see calibration data).
  - `left_vesc_id`, `right_vesc_id` (CAN IDs)
  - `can_interface` (e.g., "can0")
  - `max_velocity`, `max_angular_velocity`
  - `command_frequency` (Hz)
  - `watchdog_timeout` (s)
- **Topics:**
  - Sub: `/cmd_vel`, `/cmd_vel_real`
  - Pub: `/diagnostics`, `/cmdvel2can_debug`

---

## 3. CAN Protocol (VESC)

- **CAN ID:** `0x8000001C` (left), `0x8000002E` (right)
- **Data:** 32-bit signed integer, big-endian, `duty_cycle * 100000`
- **Frame:** Extended CAN frame (must set `CAN_EFF_FLAG`)

---

## 4. Safety System

- Node logic is only enabled if `/cmd_vel_real` is being published (monitored for freshness)
- If `/cmd_vel_real` is stale, node stops sending CAN commands

---

## 5. Implementation Plan (Single-File)

1. **Minimal Node Skeleton**
   - ROS2 node, parameter parsing, topic subscriptions
2. **SocketCAN Interface**
   - Open, send, and close CAN socket
3. **Kinematics & Duty Cycle**
   - Convert Twist to wheel velocities, then to duty cycles
4. **CAN Frame Builder**
   - Encode duty cycle as VESC CAN frame
5. **Safety Monitor**
   - Subscribe to `/cmd_vel_real`, check message freshness
6. **Main Control Loop**
   - At fixed frequency, if safe, send CAN commands
7. **Diagnostics & Logging**
   - Publish status/debug info
8. **Testing & Validation**
   - Use `candump` and ROS2 tools to verify

---

## 6. Example Launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmdvel2can',
            executable='cmdvel2can_node',
            name='cmdvel2can',
            parameters=[{
                'wheel_separation': 0.370,
                'wheel_diameter': 0.356,
                'left_vesc_id': 28,
                'right_vesc_id': 46,
                'can_interface': 'can0',
                'max_velocity': 2.0,
                'max_angular_velocity': 3.0,
                'command_frequency': 20.0,
                'watchdog_timeout': 0.5
            }]
        )
    ])
```

---

## 7. Troubleshooting

- **No movement:** Check CAN traffic with `candump can0`
- **Permission denied:** Run as root or set CAN permissions
- **Node not running:** Rebuild and source workspace

---

## 8. Future Improvements

- Add velocity ramping/limiting (behavior system)
- Add emergency stop/reset services
- Modularize for larger projects (if needed)

---

*This documentation is adapted and improved for a single-file, study-focused implementation. All logic will be in one .cpp for clarity and learning.*
