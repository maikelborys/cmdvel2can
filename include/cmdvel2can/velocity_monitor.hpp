#ifndef CMDVEL2CAN_VELOCITY_MONITOR_HPP
#define CMDVEL2CAN_VELOCITY_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <mutex>

namespace cmdvel2can {

/**
 * @brief Monitors real velocity feedback from odometry
 * 
 * This class subscribes to cmd_vel_real topic and provides
 * real-time velocity monitoring and validation capabilities.
 */
class VelocityMonitor {
public:
    /**
     * @brief Structure to hold velocity data with timestamp
     */
    struct VelocityData {
        double linear;      ///< Linear velocity (m/s)
        double angular;     ///< Angular velocity (rad/s)
        std::chrono::steady_clock::time_point timestamp;
        bool is_valid;      ///< Data validity flag
    };
    
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node for topic subscription
     */
    explicit VelocityMonitor(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Initialize velocity monitoring
     * @param topic_name Name of the velocity feedback topic (default: "cmd_vel_real")
     * @param timeout_ms Timeout for considering data stale (milliseconds)
     * @param min_frequency_hz Minimum expected message frequency (Hz)
     * @return true if initialization successful
     */
    bool initialize(const std::string& topic_name = "cmd_vel_real",
                   double timeout_ms = 1000.0,
                   double min_frequency_hz = 5.0);
    
    /**
     * @brief Check if velocity data is available and fresh
     * @return true if valid data available within timeout
     */
    bool isDataAvailable() const;
    
    /**
     * @brief Get current velocity data
     * @return VelocityData structure with current velocity and metadata
     */
    VelocityData getCurrentVelocity() const;
    
    /**
     * @brief Wait for initial velocity data with timeout
     * @param timeout_ms Maximum time to wait (milliseconds)
     * @return true if data received within timeout
     */
    bool waitForInitialData(double timeout_ms = 5000.0);
    
    /**
     * @brief Check if message frequency meets minimum requirements
     * @return true if frequency is adequate
     */
    bool isFrequencyAdequate() const;
    
    /**
     * @brief Get current message frequency
     * @return Frequency in Hz
     */
    double getCurrentFrequency() const;
    
    /**
     * @brief Get time since last message
     * @return Duration since last message
     */
    std::chrono::milliseconds getTimeSinceLastMessage() const;
    
    /**
     * @brief Enable/disable velocity monitoring
     * @param enabled true to enable, false to disable
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if monitoring is enabled
     * @return true if enabled
     */
    bool isEnabled() const;
    
    /**
     * @brief Get statistics about message reception
     */
    struct Statistics {
        uint64_t total_messages;
        uint64_t invalid_messages;
        double average_frequency;
        std::chrono::milliseconds max_gap;
        std::chrono::milliseconds min_gap;
    };
    
    /**
     * @brief Get monitoring statistics
     * @return Statistics structure
     */
    Statistics getStatistics() const;
    
    /**
     * @brief Reset statistics counters
     */
    void resetStatistics();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    
    // Configuration
    std::string topic_name_;
    std::chrono::milliseconds timeout_;
    double min_frequency_;
    bool enabled_;
    
    // State
    VelocityData current_velocity_;
    mutable std::mutex velocity_mutex_;
    
    // Statistics
    Statistics stats_;
    std::chrono::steady_clock::time_point last_message_time_;
    std::chrono::steady_clock::time_point start_time_;
    std::vector<std::chrono::milliseconds> message_intervals_;
    static constexpr size_t MAX_INTERVAL_HISTORY = 100;
    
    /**
     * @brief Callback for velocity messages
     * @param msg Received velocity message
     */
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Update frequency calculations
     */
    void updateFrequencyStats();
    
    /**
     * @brief Validate velocity message content
     * @param msg Message to validate
     * @return true if message is valid
     */
    bool validateMessage(const geometry_msgs::msg::Twist::SharedPtr msg) const;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_VELOCITY_MONITOR_HPP
