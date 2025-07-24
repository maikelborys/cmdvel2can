# Project System Overview and Development Guide

This document is the single source of truth for the current state, architecture, and development roadmap of the `cmdvel_to_can_bridge_ros2` project. It is designed to be updated as the project evolves, providing a clear view of what is implemented, what is in progress, and what is planned.

---

## 1. Project Purpose

Bridge ROS2 `/cmd_vel` messages to VESC motor controllers via CAN, enabling safe, robust, and configurable robot velocity control.

---

## 2. Architecture Overview

- **Input:** ROS2 `/cmd_vel` (geometry_msgs/Twist)
- **Output:** CAN bus messages (VESC protocol, extended frame)
- **Key Components:**
  - `CANMessageBuilder`: Encodes velocity/current/rpm to VESC CAN frames
  - `CmdVelToCANNode`: ROS2 node, subscribes to `/cmd_vel`, manages safety, publishes diagnostics
  - **Safety System:** (Planned) Monitors `/cmd_vel_real` topic; bridge only active if this topic is alive
  - **Behavior System:** (Planned) Incremental velocity changes to avoid sudden jumps

---

## 3. Feature Checklist & Status

| Feature                        | Status      | Notes                                  |
|------------------------------- |------------|----------------------------------------|
| CAN protocol (duty cycle)      | ✅ Done     | Hardware-verified, extended frame      |
| ROS2 `/cmd_vel` decoding       | ✅ Done     |                                        |
| Parameterization               | ✅ Done     | IDs, max velocity, etc.                |
| Safety system (`cmd_vel_real`) | ✅ Done     | VelocityMonitor implemented and tested |
| Behavior system (ramp/limit)   | ⬜ Planned  | Incremental velocity changes           |
| Diagnostics/Debug topics       | ✅ Done     | `/diagnostics`, `/cmd_vel_debug`       |
| Emergency stop service         | ✅ Done     | `/emergency_stop`                      |
| Reset safety service           | ✅ Done     | `/reset_safety`                        |
| Roadmap/Docs                   | ✅ Ongoing  | This file                              |

---

## 4. Usage

- **Build:** Standard ROS2 colcon build
- **Run:** Launch node with required parameters (see below)
- **Parameters:**
  - `wheel_separation`, `wheel_diameter`, `left_vesc_id`, `right_vesc_id`, `can_interface`, `command_frequency`, `watchdog_timeout`, `max_velocity`, `max_angular_velocity`
- **Topics:**
  - Sub: `/cmd_vel`, `/cmd_vel_real` (planned)
  - Pub: `/diagnostics`, `/cmd_vel_debug`
- **Services:**
  - `/emergency_stop`, `/reset_safety`

---


## 5. Safety System (Implemented)

- **Goal:** Only allow bridge to send CAN commands if `/cmd_vel_real` is being published (i.e., robot is enabled by higher-level system)
- **Implementation:**
  - `VelocityMonitor` class subscribes to `/cmd_vel_real`, tracks message freshness and frequency
  - CAN output is only enabled when fresh, valid data is available
  - Standalone test node (`velocity_monitor_test`) verifies correct operation
  - Next: Integrate VelocityMonitor into main bridge node for runtime safety enforcement

---

## 6. Behavior System (Planned)

- **Goal:** Prevent sudden velocity changes (ramps, smoothing)
- **Implementation:**
  - Compare current velocity to commanded velocity
  - If difference > threshold, incrementally adjust toward target
  - Configurable ramp rate

---

## 7. Development Workflow

- Update this document as features are added/changed
- Use the checklist above to track progress
- Add design notes and TODOs for new features
- Use clear commit messages referencing checklist items

---

## 8. Roadmap / Next Steps

- [ ] Implement safety system (`cmd_vel_real` monitoring)
- [ ] Implement behavior system (velocity ramping)
    - Next step: Add incremental velocity adjustment (ramp) logic to the main bridge node. Compare current and commanded velocity, and limit the rate of change according to a configurable ramp rate parameter. Document ramping logic and test with real hardware.
- [ ] Add launch files and parameter YAMLs
- [ ] Add more diagnostics and error recovery
- [ ] Document closed-loop control options

---

## 9. Contributing

- All new features must be documented here
- Use the checklist to mark progress
- Keep architecture and usage sections up to date

---

## 10. Known Issues / Troubleshooting

- VESC requires CAN_EFF_FLAG (extended frame) in CAN ID
- Duty cycle scaling: use *100000, not *1000
- See code comments for hardware-verified examples

---

## 11. References

- [VESC CAN Protocol](https://vesc-project.com/node/286)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [SocketCAN](https://www.kernel.org/doc/Documentation/networking/can.txt)

---

_Last updated: 2025-07-23_
