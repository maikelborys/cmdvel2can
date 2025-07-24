#ifndef CMDVEL2CAN_CMDVEL_TO_CAN_NODE_HPP
#define CMDVEL2CAN_CMDVEL_TO_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <can_msgs/msg/frame.hpp>

#include "cmdvel2can/can_interface.hpp"
#include "cmdvel2can/velocity_converter.hpp"
#include "cmdvel2can/can_message_builder.hpp"
#include "cmdvel2can/safety_system.hpp"
#include "cmdvel2can/velocity_monitor.hpp"

#include <memory>
#include <chrono>

namespace cmdvel2can {

/**
 * @brief Main ROS2 node for cmd_vel to CAN bridge
 * 
 * This node subscribes to cmd_vel messages and converts them to
 * CAN commands for VESC motor controllers, with integrated safety systems.
 */
class CmdVelToCANNode : public rclcpp::Node {
public:
    void initialize();
public:
    /**
     * @brief Constructor
     */
    CmdVelToCANNode();
    
    /**
     * @brief Destructor
     */
    ~CmdVelToCANNode();

private:
    // Safety: VelocityMonitor for /cmd_vel_real
    std::unique_ptr<VelocityMonitor> velocity_monitor_;
    /**
     * @brief Callback for cmd_vel messages
     * @param msg Received Twist message
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Timer callback for periodic operations
     */
    void timerCallback();
    
    /**
     * @brief Publish diagnostic information
     */
    void publishDiagnostics();
    
    /**
     * @brief Emergency stop service callback
     * @param request Service request
     * @param response Service response
     */
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Reset safety service callback
     * @param request Service request
     * @param response Service response
     */
    void resetSafetyCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Initialize node parameters
     */
    void initializeParameters();
    
    /**
     * @brief Initialize all components
     * @return true if successful, false otherwise
     */
    bool initializeComponents();
    
    /**
     * @brief Send stop commands to both VESCs
     */
    void sendStopCommands();
    
    /**
     * @brief Process velocity command and send CAN messages
     * @param linear Linear velocity (m/s)
     * @param angular Angular velocity (rad/s)
     */
    void processVelocityCommand(double linear, double angular);
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_debug_pub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr liveness_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_safety_service_;

    // CAN publishing helper
    void sendCanMessage(double left_wheel_velocity, double right_wheel_velocity);
    
    // Core components
    std::unique_ptr<CANInterface> can_interface_;
    std::unique_ptr<VelocityConverter> velocity_converter_;
    std::unique_ptr<CANMessageBuilder> message_builder_;
    std::unique_ptr<SafetySystem> safety_system_;
    
    // Parameters
    double wheel_separation_;
    double wheel_diameter_;
    double max_velocity_;
    double max_angular_velocity_;
    uint8_t left_vesc_id_;
    uint8_t right_vesc_id_;
    std::string can_interface_name_;
    double command_frequency_;
    double watchdog_timeout_;
    bool debug_mode_;
    bool publish_diagnostics_;
    
    // State
    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    bool is_initialized_;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_CMDVEL_TO_CAN_NODE_HPP
