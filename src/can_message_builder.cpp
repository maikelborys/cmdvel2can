// Implementation for VESC CAN message builder
// Based on real hardware testing with VESC IDs 28 (0x1C) and 46 (0x2E)

#include "cmdvel2can/can_message_builder.hpp"
#include <cstring>
#include <algorithm>
#include <linux/can.h>

namespace cmdvel2can {

CANMessageBuilder::CANMessageBuilder(uint8_t left_vesc_id, uint8_t right_vesc_id)
    : left_vesc_id_(left_vesc_id), right_vesc_id_(right_vesc_id) {
    // Verified VESC IDs from hardware testing:
    // Left VESC: ID 28 (0x1C) → CAN ID 0x0000001C
    // Right VESC: ID 46 (0x2E) → CAN ID 0x0000002E
}

can_frame CANMessageBuilder::buildDutyCycleCommand(uint8_t vesc_id, double duty_cycle) {
    // Clamp duty cycle to safe bounds
    duty_cycle = std::clamp(duty_cycle, -1.0, 1.0);
    
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    // Set CAN ID with extended frame format (like cansend does)
    frame.can_id = static_cast<uint32_t>(vesc_id) | CAN_EFF_FLAG;
    frame.can_dlc = 4;  // VESC duty cycle commands are 4 bytes
    
    // Scale duty cycle based on real VESC examples:
    // The VESC protocol uses percentage * 1000, not percentage * 100000
    // Examples from hardware testing:
    // duty_cycle = -0.10 (-10%) → -10000 → 0xFF.FF.D8.F0 
    // duty_cycle = -0.046(-4.6%) → -4576 → 0xFF.FF.EE.20 (your max test)
    // duty_cycle =  0.0  (0%)   → 0      → 0x00.00.00.00 
    // duty_cycle =  0.06 (6%)   → 6000   → 0x00.00.17.70 
    // duty_cycle =  0.10 (10%)  → 10000  → 0x00.00.27.10
    int32_t vesc_duty = static_cast<int32_t>(duty_cycle * 100000);
    
    // Pack as big-endian 32-bit signed integer
    frame.data[0] = (vesc_duty >> 24) & 0xFF;
    frame.data[1] = (vesc_duty >> 16) & 0xFF;
    frame.data[2] = (vesc_duty >> 8) & 0xFF;
    frame.data[3] = vesc_duty & 0xFF;
    
    return frame;
}

can_frame CANMessageBuilder::buildVelocityCommand(uint8_t vesc_id, double velocity) {
    // Convert velocity to duty cycle (requires max_velocity parameter)
    // This is a placeholder - actual implementation needs max velocity configuration
    // For now, assume max velocity of 2.0 m/s corresponds to 100% duty cycle
    double duty_cycle = velocityToDutyCycle(velocity, 2.0);
    return buildDutyCycleCommand(vesc_id, duty_cycle);
}

can_frame CANMessageBuilder::buildCurrentCommand(uint8_t vesc_id, double current) {
    int32_t vesc_current = scaleCurrent(current);
    return buildVescCommand(vesc_id, CommandType::SET_CURRENT, vesc_current);
}

can_frame CANMessageBuilder::buildRPMCommand(uint8_t vesc_id, double rpm) {
    int32_t vesc_rpm = scaleRPM(rpm);
    return buildVescCommand(vesc_id, CommandType::SET_RPM, vesc_rpm);
}

double CANMessageBuilder::velocityToDutyCycle(double velocity, double max_velocity) {
    // Empirical calibration (2025): duty_cycle = velocity_mps / 7.857
    // See DESIGN_AND_API.md for details
    if (max_velocity <= 0.0) return 0.0;
    return std::clamp(velocity / 7.857, -1.0, 1.0);
}

double CANMessageBuilder::dutyCycleToVelocity(double duty_cycle, double max_velocity) {
    return duty_cycle * max_velocity;
}

uint8_t CANMessageBuilder::getLeftVescId() const {
    return left_vesc_id_;
}

uint8_t CANMessageBuilder::getRightVescId() const {
    return right_vesc_id_;
}

bool CANMessageBuilder::isValidDutyCycle(double duty_cycle) {
    return (duty_cycle >= -1.0) && (duty_cycle <= 1.0);
}

can_frame CANMessageBuilder::buildVescCommand(uint8_t vesc_id, CommandType command_type, int32_t value) {
    (void)command_type; // Suppress unused parameter warning
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    // Set CAN ID based on VESC ID
    // Format: 0x000000XX where XX is the VESC ID
    frame.can_id = static_cast<uint32_t>(vesc_id);
    // Set data length
    frame.can_dlc = 4;  // 4 bytes for duty cycle command
    // Pack 32-bit value in big-endian format (verified from hardware)
    frame.data[0] = (value >> 24) & 0xFF;  // Most significant byte
    frame.data[1] = (value >> 16) & 0xFF;
    frame.data[2] = (value >> 8) & 0xFF;
    frame.data[3] = value & 0xFF;          // Least significant byte
    return frame;
}

int32_t CANMessageBuilder::scaleDutyCycle(double duty_cycle) const {
    // Convert normalized duty cycle to VESC format
    // -1.0 to 1.0 → -100000 to 100000
    // 
    // Examples from hardware testing:
    // duty_cycle = -0.1 → -10000 → 0xFFFFD8F0
    // duty_cycle = 0.0  → 0      → 0x00000000  
    // duty_cycle = 0.1  → 10000  → 0x00002710
    
    return static_cast<int32_t>(duty_cycle * DUTY_CYCLE_SCALE);
}

int32_t CANMessageBuilder::scaleCurrent(double current) const {
    // Scale current from amperes to VESC format
    // Implementation needed based on VESC current command format
    return static_cast<int32_t>(current * CURRENT_SCALE);
}

int32_t CANMessageBuilder::scaleRPM(double rpm) const {
    // Scale RPM to VESC format
    // Implementation needed based on VESC RPM command format
    return static_cast<int32_t>(rpm * RPM_SCALE);
}

} // namespace cmdvel2can
