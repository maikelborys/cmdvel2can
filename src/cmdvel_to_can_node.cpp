// ROS2 node implementation for CmdVel to CAN Bridge
// Handles cmd_vel subscription and VESC CAN command transmission

#include "cmdvel2can/cmdvel_to_can_bridge.hpp"
#include <algorithm>
#include <chrono>

namespace cmdvel2can {

// TODO: Implement CmdVelToCANNode class
// This file will contain:
// - ROS2 node lifecycle management
// - Topic subscription and publishing
// - Service handling
// - Component orchestration

CmdVelToCANNode::CmdVelToCANNode()
    : Node("cmdvel_to_can_bridge"), is_initialized_(false) {
    
    RCLCPP_INFO(get_logger(), "Initializing CmdVel to CAN Bridge Node");
    
    // Initialize parameters
    initializeParameters();
    
    // Initialize components
    if (!initializeComponents()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize components");
        return;
    }
    
    // Create subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelToCANNode::cmdVelCallback, this, std::placeholders::_1));
    
    // Create publishers
    if (publish_diagnostics_) {
        diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "diagnostics", 10);
    }
    
    if (debug_mode_) {
        cmd_vel_debug_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_debug", 10);
    }
    
    // Create services
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        std::bind(&CmdVelToCANNode::emergencyStopCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    reset_safety_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_safety",
        std::bind(&CmdVelToCANNode::resetSafetyCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Create timer for periodic operations
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / command_frequency_)),
        std::bind(&CmdVelToCANNode::timerCallback, this));
    
    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "CmdVel to CAN Bridge Node initialized successfully");
}

CmdVelToCANNode::~CmdVelToCANNode() {
    RCLCPP_INFO(get_logger(), "Shutting down CmdVel to CAN Bridge Node");
    sendStopCommands();
    if (can_interface_) {
        can_interface_->shutdown();
    }
}

void CmdVelToCANNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!is_initialized_ || !can_interface_ || !message_builder_) {
        RCLCPP_WARN(get_logger(), "Components not initialized, ignoring cmd_vel");
        return;
    }
    
    // Update last command time
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    
    // For testing: convert directly to duty cycle (simple proportional)
    // This is a basic implementation - proper differential drive will be added later
    double linear = msg->linear.x;   // m/s
    double angular = msg->angular.z; // rad/s
    
    // Clamp to safe limits
    linear = std::clamp(linear, -max_velocity_, max_velocity_);
    angular = std::clamp(angular, -max_angular_velocity_, max_angular_velocity_);
    
    // Simple differential drive approximation for testing
    // TODO: Replace with proper VelocityConverter implementation
    double wheel_radius = wheel_diameter_ / 2.0;
    double left_velocity = linear - (angular * wheel_separation_ / 2.0);
    double right_velocity = linear + (angular * wheel_separation_ / 2.0);
    
    // Convert to duty cycle to match your working test (-2.56%)
    // Your test: 0.1 m/s should give us ~2.5% duty cycle  
    double max_test_velocity = 0.1;  // Very conservative for testing
    double max_duty_cycle = 0.025;  // 2.5% max to match your working test
    double left_duty = (left_velocity / max_test_velocity) * max_duty_cycle;
    double right_duty = (right_velocity / max_test_velocity) * max_duty_cycle;
    
    // Build and send CAN messages
    can_frame left_frame = message_builder_->buildDutyCycleCommand(left_vesc_id_, left_duty);
    can_frame right_frame = message_builder_->buildDutyCycleCommand(right_vesc_id_, right_duty);
    
    if (can_interface_->sendFrame(left_frame)) {
        RCLCPP_INFO(get_logger(), "✓ Sent left VESC command: duty=%.3f", left_duty);
    } else {
        RCLCPP_ERROR(get_logger(), "✗ Failed to send left VESC command");
    }
    
    if (can_interface_->sendFrame(right_frame)) {
        RCLCPP_INFO(get_logger(), "✓ Sent right VESC command: duty=%.3f", right_duty);
    } else {
        RCLCPP_ERROR(get_logger(), "✗ Failed to send right VESC command");
    }
    
    // Log for debugging
    RCLCPP_INFO(get_logger(), "cmd_vel: linear=%.2f, angular=%.2f → left_duty=%.3f, right_duty=%.3f", 
                linear, angular, left_duty, right_duty);
}

void CmdVelToCANNode::timerCallback() {
    // Implementation needed
}

void CmdVelToCANNode::publishDiagnostics() {
    // Implementation needed
}

void CmdVelToCANNode::emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Implementation needed
}

void CmdVelToCANNode::resetSafetyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Implementation needed
}

void CmdVelToCANNode::initializeParameters() {
    RCLCPP_INFO(get_logger(), "Loading parameters...");
    
    // Declare parameters with defaults
    this->declare_parameter("wheel_separation", 0.370);
    this->declare_parameter("wheel_diameter", 0.3556);
    this->declare_parameter("left_vesc_id", 28);
    this->declare_parameter("right_vesc_id", 46);
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("max_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 3.0);
    this->declare_parameter("command_frequency", 20.0);
    this->declare_parameter("watchdog_timeout", 0.5);
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("publish_diagnostics", true);
    
    // Get parameter values
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
    left_vesc_id_ = this->get_parameter("left_vesc_id").as_int();
    right_vesc_id_ = this->get_parameter("right_vesc_id").as_int();
    can_interface_name_ = this->get_parameter("can_interface").as_string();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    command_frequency_ = this->get_parameter("command_frequency").as_double();
    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();
    
    RCLCPP_INFO(get_logger(), "Parameters loaded:");
    RCLCPP_INFO(get_logger(), "  - Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(get_logger(), "  - Wheel diameter: %.3f m", wheel_diameter_);
    RCLCPP_INFO(get_logger(), "  - Left VESC ID: %d", left_vesc_id_);
    RCLCPP_INFO(get_logger(), "  - Right VESC ID: %d", right_vesc_id_);
    RCLCPP_INFO(get_logger(), "  - CAN interface: %s", can_interface_name_.c_str());
    RCLCPP_INFO(get_logger(), "  - Max velocity: %.2f m/s", max_velocity_);
    RCLCPP_INFO(get_logger(), "  - Max angular velocity: %.2f rad/s", max_angular_velocity_);
    RCLCPP_INFO(get_logger(), "  - Command frequency: %.1f Hz", command_frequency_);
    RCLCPP_INFO(get_logger(), "  - Watchdog timeout: %.2f s", watchdog_timeout_);
}

bool CmdVelToCANNode::initializeComponents() {
    RCLCPP_INFO(get_logger(), "Initializing components...");
    
    // Initialize CAN interface
    can_interface_ = std::make_unique<CANInterface>(can_interface_name_);
    if (!can_interface_->initialize()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize CAN interface");
        return false;
    }
    
    // Initialize message builder with VESC IDs
    message_builder_ = std::make_unique<CANMessageBuilder>(left_vesc_id_, right_vesc_id_);
    
    // TODO: Initialize velocity converter
    // velocity_converter_ = std::make_unique<VelocityConverter>(wheel_separation_, wheel_diameter_);
    
    // TODO: Initialize safety system
    // safety_system_ = std::make_unique<SafetySystem>(watchdog_timeout_);
    
    RCLCPP_INFO(get_logger(), "Components initialized successfully");
    RCLCPP_INFO(get_logger(), "CAN interface: %s, VESC IDs: %d, %d", 
                can_interface_name_.c_str(), left_vesc_id_, right_vesc_id_);
    
    return true;
}

void CmdVelToCANNode::sendStopCommands() {
    if (!can_interface_ || !message_builder_) {
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Sending stop commands to VESCs");
    
    // Send zero duty cycle to both VESCs
    can_frame left_stop = message_builder_->buildDutyCycleCommand(left_vesc_id_, 0.0);
    can_frame right_stop = message_builder_->buildDutyCycleCommand(right_vesc_id_, 0.0);
    
    can_interface_->sendFrame(left_stop);
    can_interface_->sendFrame(right_stop);
}

void CmdVelToCANNode::processVelocityCommand(double linear, double angular) {
    // Implementation needed
}

} // namespace cmdvel2can
