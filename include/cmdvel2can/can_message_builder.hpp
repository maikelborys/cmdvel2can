#ifndef CMDVEL2CAN_CAN_MESSAGE_BUILDER_HPP
#define CMDVEL2CAN_CAN_MESSAGE_BUILDER_HPP

#include <linux/can.h>
#include <cstdint>

namespace cmdvel2can {

/**
 * @brief Builds CAN messages for VESC motor controllers
 * 
 * This class handles the VESC CAN protocol for sending duty cycle,
 * current, and RPM commands to VESC motor controllers.
 * Based on real hardware testing with VESC IDs 28 and 46.
 */
class CANMessageBuilder {
public:
    /**
     * @brief VESC CAN command types (verified from hardware)
     */
    enum class CommandType {
        SET_DUTY_CYCLE = 0x00,     ///< Set duty cycle (-100% to +100%)
        SET_CURRENT = 0x01,        ///< Set current (amperes)
        SET_CURRENT_BRAKE = 0x02,  ///< Set braking current
        SET_RPM = 0x03,            ///< Set RPM (revolutions per minute)
        SET_POS = 0x04             ///< Set position
    };
    
    // VESC Protocol Constants (verified from hardware testing)
    static constexpr int32_t DUTY_CYCLE_SCALE = 100000;  // -100000 to +100000 for -100% to +100%
    static constexpr int32_t CURRENT_SCALE = 1000;       // Scale factor for current commands
    static constexpr int32_t RPM_SCALE = 1;              // Scale factor for RPM commands
    
    /**
     * @brief Constructor
     * @param left_vesc_id CAN ID of the left VESC (default: 28 = 0x1C)
     * @param right_vesc_id CAN ID of the right VESC (default: 46 = 0x2E)
     */
    CANMessageBuilder(uint8_t left_vesc_id, uint8_t right_vesc_id);
    
    /**
     * @brief Build duty cycle command for a VESC (PRIMARY COMMAND TYPE)
     * @param vesc_id Target VESC CAN ID
     * @param duty_cycle Desired duty cycle (-1.0 to +1.0, representing -100% to +100%)
     * @return CAN frame ready for transmission
     * 
     * Example: duty_cycle = 0.1 → 10% → 0x00002710 in CAN message
     */
    can_frame buildDutyCycleCommand(uint8_t vesc_id, double duty_cycle);
    
    /**
     * @brief Build velocity command for a VESC (converts to duty cycle)
     * @param vesc_id Target VESC CAN ID
     * @param velocity Desired velocity in m/s
     * @return CAN frame ready for transmission
     */
    can_frame buildVelocityCommand(uint8_t vesc_id, double velocity);
    
    /**
     * @brief Build current command for a VESC
     * @param vesc_id Target VESC CAN ID  
     * @param current Desired current in amperes
     * @return CAN frame ready for transmission
     */
    can_frame buildCurrentCommand(uint8_t vesc_id, double current);
    
    /**
     * @brief Build RPM command for a VESC
     * @param vesc_id Target VESC CAN ID
     * @param rpm Desired RPM
     * @return CAN frame ready for transmission
     */
    can_frame buildRPMCommand(uint8_t vesc_id, double rpm);
    
    /**
     * @brief Convert velocity to duty cycle based on wheel parameters
     * @param velocity Linear velocity in m/s
     * @param max_velocity Maximum velocity for 100% duty cycle
     * @return Duty cycle value (-1.0 to 1.0)
     */
    static double velocityToDutyCycle(double velocity, double max_velocity);
    
    /**
     * @brief Convert duty cycle to velocity based on wheel parameters
     * @param duty_cycle Duty cycle (-1.0 to 1.0)
     * @param max_velocity Maximum velocity at 100% duty cycle
     * @return Linear velocity in m/s
     */
    static double dutyCycleToVelocity(double duty_cycle, double max_velocity);
    
    /**
     * @brief Get left VESC ID
     * @return Left VESC CAN ID
     */
    uint8_t getLeftVescId() const;
    
    /**
     * @brief Get right VESC ID
     * @return Right VESC CAN ID
     */
    uint8_t getRightVescId() const;
    
    /**
     * @brief Validate duty cycle is within safe bounds
     * @param duty_cycle Duty cycle to validate
     * @return true if within bounds (-1.0 to 1.0)
     */
    static bool isValidDutyCycle(double duty_cycle);

private:
    uint8_t left_vesc_id_;   ///< CAN ID for left VESC (28 = 0x1C)
    uint8_t right_vesc_id_;  ///< CAN ID for right VESC (46 = 0x2E)
    
    /**
     * @brief Build generic VESC command frame
     * @param vesc_id Target VESC CAN ID
     * @param command_type Type of command
     * @param value Command value (scaled appropriately)
     * @return CAN frame ready for transmission
     */
    can_frame buildVescCommand(uint8_t vesc_id, CommandType command_type, int32_t value);
    
    /**
     * @brief Scale duty cycle from normalized (-1.0 to 1.0) to VESC format
     * @param duty_cycle Normalized duty cycle
     * @return VESC-formatted duty cycle (-100000 to 100000)
     */
    int32_t scaleDutyCycle(double duty_cycle) const;
    
    /**
     * @brief Scale current from amperes to VESC format
     * @param current Current in amperes
     * @return VESC-formatted current value
     */
    int32_t scaleCurrent(double current) const;
    
    /**
     * @brief Scale RPM to VESC format
     * @param rpm RPM value
     * @return VESC-formatted RPM value
     */
    int32_t scaleRPM(double rpm) const;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_CAN_MESSAGE_BUILDER_HPP
