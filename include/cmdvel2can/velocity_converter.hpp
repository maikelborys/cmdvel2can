#ifndef CMDVEL2CAN_VELOCITY_CONVERTER_HPP
#define CMDVEL2CAN_VELOCITY_CONVERTER_HPP

#include <geometry_msgs/msg/twist.hpp>

namespace cmdvel2can {

/**
 * @brief Converts cmd_vel messages to individual wheel velocities
 * 
 * This class handles the differential drive kinematics to convert
 * linear and angular velocities to left and right wheel velocities.
 */
class VelocityConverter {
public:
    /**
     * @brief Structure to hold wheel velocities
     */
    struct WheelVelocities {
        double left;   ///< Left wheel velocity (m/s)
        double right;  ///< Right wheel velocity (m/s)
    };
    
    /**
     * @brief Constructor
     * @param wheel_separation Distance between wheel centers (meters)
     * @param wheel_diameter Diameter of the wheels (meters)
     */
    VelocityConverter(double wheel_separation, double wheel_diameter);
    
    /**
     * @brief Convert cmd_vel to wheel velocities
     * @param cmd_vel ROS2 Twist message with linear and angular velocities
     * @return WheelVelocities structure with left and right wheel speeds
     */
    WheelVelocities convertCmdVel(const geometry_msgs::msg::Twist& cmd_vel);
    
    /**
     * @brief Validate that wheel velocities are within acceptable limits
     * @param velocities Wheel velocities to validate
     * @return true if velocities are valid, false otherwise
     */
    bool validateVelocities(const WheelVelocities& velocities) const;
    
    /**
     * @brief Clamp wheel velocities to maximum allowed values
     * @param velocities Wheel velocities to clamp (modified in place)
     */
    void clampVelocities(WheelVelocities& velocities) const;
    
    /**
     * @brief Set maximum allowed wheel velocity
     * @param max_velocity Maximum wheel velocity (m/s)
     */
    void setMaxVelocity(double max_velocity);
    
    /**
     * @brief Get current maximum velocity setting
     * @return Maximum velocity (m/s)
     */
    double getMaxVelocity() const;
    
    /**
     * @brief Get wheel separation
     * @return Wheel separation distance (meters)
     */
    double getWheelSeparation() const;
    
    /**
     * @brief Get wheel radius
     * @return Wheel radius (meters)
     */
    double getWheelRadius() const;
    
private:
    double wheel_separation_;  ///< Distance between wheel centers (m)
    double wheel_radius_;      ///< Wheel radius (m)
    double max_velocity_;      ///< Maximum allowed wheel velocity (m/s)
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_VELOCITY_CONVERTER_HPP
