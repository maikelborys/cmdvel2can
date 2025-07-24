#ifndef CMDVEL2CAN_SAFETY_SYSTEM_HPP
#define CMDVEL2CAN_SAFETY_SYSTEM_HPP

#include <chrono>
#include <mutex>
#include <atomic>

namespace cmdvel2can {

/**
 * @brief Safety system for monitoring and protecting robot operation
 * 
 * This class implements various safety mechanisms including watchdog timers,
 * velocity validation, and emergency stop functionality.
 */
class SafetySystem {
public:
    /**
     * @brief Safety system states
     */
    enum class SafetyState {
        NORMAL,         ///< Normal operation
        WARNING,        ///< Warning condition detected
        EMERGENCY_STOP, ///< Emergency stop activated
        FAULT          ///< System fault detected
    };
    
    /**
     * @brief Constructor
     */
    SafetySystem();
    
    /**
     * @brief Initialize safety system with configuration
     * @param watchdog_timeout_ms Watchdog timeout in milliseconds
     * @param velocity_timeout_ms Velocity timeout in milliseconds
     * @param max_linear_velocity Maximum allowed linear velocity (m/s)
     * @param max_angular_velocity Maximum allowed angular velocity (rad/s)
     */
    void initialize(double watchdog_timeout_ms, 
                   double velocity_timeout_ms,
                   double max_linear_velocity,
                   double max_angular_velocity);
    
    /**
     * @brief Check if linear velocity is within safe limits
     * @param linear Linear velocity to check (m/s)
     * @return true if safe, false otherwise
     */
    bool checkLinearVelocityLimits(double linear) const;
    
    /**
     * @brief Check if angular velocity is within safe limits
     * @param angular Angular velocity to check (rad/s)
     * @return true if safe, false otherwise
     */
    bool checkAngularVelocityLimits(double angular) const;
    
    /**
     * @brief Check both linear and angular velocity limits
     * @param linear Linear velocity (m/s)
     * @param angular Angular velocity (rad/s)
     * @return true if both are safe, false otherwise
     */
    bool checkVelocityLimits(double linear, double angular) const;
    
    /**
     * @brief Check if watchdog timer has expired
     * @return true if watchdog is healthy, false if expired
     */
    bool checkWatchdog() const;
    
    /**
     * @brief Update watchdog timer (call when valid command received)
     */
    void updateWatchdog();
    
    /**
     * @brief Trigger emergency stop
     * @param reason Description of why emergency stop was triggered
     */
    void triggerEmergencyStop(const std::string& reason);
    
    /**
     * @brief Reset emergency stop state
     * @return true if successfully reset, false if conditions not met
     */
    bool resetEmergencyStop();
    
    /**
     * @brief Get current safety state
     * @return Current SafetyState
     */
    SafetyState getCurrentState() const;
    
    /**
     * @brief Get human-readable state description
     * @return String description of current state
     */
    std::string getStateDescription() const;
    
    /**
     * @brief Get last emergency stop reason
     * @return String describing the last emergency stop reason
     */
    std::string getLastEmergencyReason() const;
    
    /**
     * @brief Check if system is in safe state for operation
     * @return true if safe to operate, false otherwise
     */
    bool isSafeToOperate() const;
    
    /**
     * @brief Enable/disable watchdog functionality
     * @param enable true to enable, false to disable
     */
    void setWatchdogEnabled(bool enable);
    
    /**
     * @brief Enable/disable emergency stop functionality
     * @param enable true to enable, false to disable
     */
    void setEmergencyStopEnabled(bool enable);
    
    /**
     * @brief Update velocity limits
     * @param max_linear Maximum linear velocity (m/s)
     * @param max_angular Maximum angular velocity (rad/s)
     */
    void updateVelocityLimits(double max_linear, double max_angular);
    
    /**
     * @brief Get time since last watchdog update
     * @return Duration since last update in milliseconds
     */
    std::chrono::milliseconds getTimeSinceLastUpdate() const;
    
private:
    // Configuration
    std::chrono::milliseconds watchdog_timeout_;
    std::chrono::milliseconds velocity_timeout_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // State
    std::atomic<SafetyState> current_state_;
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point last_velocity_time_;
    std::string last_emergency_reason_;
    
    // Flags
    std::atomic<bool> watchdog_enabled_;
    std::atomic<bool> emergency_stop_enabled_;
    std::atomic<bool> velocity_limits_enabled_;
    
    // Thread safety
    mutable std::mutex safety_mutex_;
    
    /**
     * @brief Update internal safety state based on current conditions
     */
    void updateSafetyState();
    
    /**
     * @brief Check if enough time has passed since last velocity command
     * @return true if timeout exceeded, false otherwise
     */
    bool checkVelocityTimeout() const;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_SAFETY_SYSTEM_HPP
