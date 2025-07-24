#ifndef CMDVEL2CAN_INCREMENTAL_CONTROLLER_HPP
#define CMDVEL2CAN_INCREMENTAL_CONTROLLER_HPP

#include <chrono>
#include <functional>

namespace cmdvel2can {

/**
 * @brief Controls incremental velocity adjustments for smooth motor transitions
 * 
 * This class implements various acceleration curves and safety mechanisms
 * to prevent abrupt velocity changes that could damage motors or controllers.
 */
class IncrementalController {
public:
    /**
     * @brief Acceleration curve types
     */
    enum class CurveType {
        LINEAR,      ///< Linear acceleration/deceleration
        EXPONENTIAL, ///< Exponential smoothing
        S_CURVE,     ///< S-curve (sigmoid) for smooth transitions
        CUSTOM       ///< User-defined curve function
    };
    
    /**
     * @brief Structure for velocity command with metadata
     */
    struct VelocityCommand {
        double linear;      ///< Target linear velocity (m/s)
        double angular;     ///< Target angular velocity (rad/s)
        std::chrono::steady_clock::time_point timestamp;
        bool is_emergency_stop; ///< Emergency stop flag
    };
    
    /**
     * @brief Structure for incremental output
     */
    struct IncrementalOutput {
        double linear;      ///< Adjusted linear velocity (m/s)
        double angular;     ///< Adjusted angular velocity (rad/s)
        bool target_reached; ///< True if target velocity achieved
        double progress;    ///< Progress towards target (0.0 to 1.0)
    };
    
    /**
     * @brief Constructor
     */
    IncrementalController();
    
    /**
     * @brief Initialize controller with configuration
     * @param max_linear_delta Maximum linear velocity change per cycle (m/s)
     * @param max_angular_delta Maximum angular velocity change per cycle (rad/s)
     * @param curve_type Type of acceleration curve to use
     * @param control_frequency Controller update frequency (Hz)
     */
    void initialize(double max_linear_delta,
                   double max_angular_delta,
                   CurveType curve_type = CurveType::S_CURVE,
                   double control_frequency = 50.0);
    
    /**
     * @brief Set new target velocity
     * @param target Target velocity command
     * @param current_velocity Current actual velocity
     */
    void setTarget(const VelocityCommand& target, 
                   const VelocityCommand& current_velocity);
    
    /**
     * @brief Calculate next incremental step
     * @param current_velocity Current actual velocity
     * @return IncrementalOutput with adjusted velocities
     */
    IncrementalOutput calculateIncrement(const VelocityCommand& current_velocity);
    
    /**
     * @brief Check if velocity change is safe
     * @param current Current velocity
     * @param target Target velocity
     * @return true if change is safe, false otherwise
     */
    bool isSafeTransition(const VelocityCommand& current, 
                         const VelocityCommand& target) const;
    
    /**
     * @brief Trigger emergency stop
     * @param deceleration Emergency deceleration rate (m/s²)
     */
    void emergencyStop(double deceleration = 2.0);
    
    /**
     * @brief Check if emergency stop is active
     * @return true if emergency stop engaged
     */
    bool isEmergencyStopActive() const;
    
    /**
     * @brief Reset emergency stop state
     */
    void resetEmergencyStop();
    
    /**
     * @brief Set maximum velocity deltas
     * @param max_linear_delta Maximum linear change per cycle
     * @param max_angular_delta Maximum angular change per cycle
     */
    void setMaxDeltas(double max_linear_delta, double max_angular_delta);
    
    /**
     * @brief Set acceleration curve type
     * @param curve_type New curve type
     */
    void setCurveType(CurveType curve_type);
    
    /**
     * @brief Set custom curve function
     * @param curve_function Function that takes progress (0-1) and returns scaling factor (0-1)
     */
    void setCustomCurve(std::function<double(double)> curve_function);
    
    /**
     * @brief Set velocity tolerance for target detection
     * @param linear_tolerance Linear velocity tolerance (m/s)
     * @param angular_tolerance Angular velocity tolerance (rad/s)
     */
    void setTolerances(double linear_tolerance, double angular_tolerance);
    
    /**
     * @brief Get current target velocity
     * @return Current target command
     */
    VelocityCommand getCurrentTarget() const;
    
    /**
     * @brief Get progress towards current target
     * @return Progress value (0.0 to 1.0)
     */
    double getProgress() const;
    
    /**
     * @brief Check if target has been reached
     * @return true if at target velocity within tolerance
     */
    bool isAtTarget() const;
    
    /**
     * @brief Enable/disable safety checks
     * @param enabled true to enable safety validation
     */
    void setSafetyEnabled(bool enabled);
    
    /**
     * @brief Set maximum allowed direction change rate
     * @param max_rate Maximum rate of direction change (m/s per second)
     */
    void setMaxDirectionChangeRate(double max_rate);

private:
    // Configuration
    double max_linear_delta_;
    double max_angular_delta_;
    CurveType curve_type_;
    double control_frequency_;
    double control_period_;
    
    // Tolerances
    double linear_tolerance_;
    double angular_tolerance_;
    
    // Safety parameters
    bool safety_enabled_;
    double max_direction_change_rate_;
    
    // State
    VelocityCommand target_velocity_;
    VelocityCommand previous_output_;
    std::chrono::steady_clock::time_point last_update_time_;
    bool emergency_stop_active_;
    double emergency_deceleration_;
    
    // Custom curve function
    std::function<double(double)> custom_curve_function_;
    
    /**
     * @brief Apply acceleration curve to progress value
     * @param progress Raw progress (0.0 to 1.0)
     * @return Curved progress value
     */
    double applyCurve(double progress) const;
    
    /**
     * @brief Calculate linear acceleration curve
     * @param progress Input progress
     * @return Scaled progress
     */
    double linearCurve(double progress) const;
    
    /**
     * @brief Calculate exponential acceleration curve
     * @param progress Input progress
     * @return Scaled progress
     */
    double exponentialCurve(double progress) const;
    
    /**
     * @brief Calculate S-curve acceleration curve
     * @param progress Input progress
     * @return Scaled progress
     */
    double sCurve(double progress) const;
    
    /**
     * @brief Validate direction change rate
     * @param current Current velocity
     * @param target Target velocity
     * @return true if change rate is acceptable
     */
    bool validateDirectionChangeRate(const VelocityCommand& current,
                                   const VelocityCommand& target) const;
    
    /**
     * @brief Calculate time-based progress towards target
     * @param current Current velocity
     * @return Progress value (0.0 to 1.0)
     */
    double calculateProgress(const VelocityCommand& current) const;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_INCREMENTAL_CONTROLLER_HPP
