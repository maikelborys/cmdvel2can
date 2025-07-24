
#include "cmdvel2can/cmdvel_to_can_bridge.hpp"
#include "cmdvel2can/velocity_monitor.hpp"
using namespace cmdvel2can;

namespace cmdvel2can {

CmdVelToCANNode::~CmdVelToCANNode() = default;



CmdVelToCANNode::CmdVelToCANNode() : Node("cmdvel_to_can_bridge") {

    this->declare_parameter("max_velocity", 1.0);
    this->declare_parameter("wheel_separation", 0.5);

    this->get_parameter("max_velocity", max_velocity_);
    this->get_parameter("wheel_separation", wheel_separation_);
}

void CmdVelToCANNode::initialize() {
    // Strictly require /cmd_vel_real for safety
    velocity_monitor_ = std::make_unique<VelocityMonitor>(shared_from_this());
    velocity_monitor_->initialize("cmd_vel_real", 1000.0, 1.0); // 1s timeout, 1Hz min freq

    RCLCPP_INFO(this->get_logger(), "Waiting for /cmd_vel_real data at startup...");
    auto start = std::chrono::steady_clock::now();
    bool got_data = false;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < 5000) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (velocity_monitor_->isDataAvailable()) {
            got_data = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!got_data) {
        RCLCPP_FATAL(this->get_logger(), "Startup safety: /cmd_vel_real not available after 5s. Shutting down node.");
        rclcpp::shutdown();
        std::exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->get_logger(), "/cmd_vel_real data received. Node startup continues.");

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CmdVelToCANNode::cmdVelCallback, this, std::placeholders::_1));

    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
    
    // Timer to monitor /cmd_vel_real liveness
    liveness_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
            if (!velocity_monitor_ || !velocity_monitor_->isDataAvailable()) {
                RCLCPP_FATAL(this->get_logger(), "Runtime safety: /cmd_vel_real lost or stale. Shutting down node.");
                rclcpp::shutdown();
            }
        }
    );
}

void CmdVelToCANNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Only allow CAN output if /cmd_vel_real is fresh and valid
    bool monitor_exists = (velocity_monitor_ != nullptr);
    bool data_available = monitor_exists ? velocity_monitor_->isDataAvailable() : false;
    RCLCPP_INFO(this->get_logger(), "[DEBUG] VelocityMonitor exists: %s, isDataAvailable: %s", monitor_exists ? "true" : "false", data_available ? "true" : "false");
    if (!monitor_exists || !data_available) {
        RCLCPP_WARN(this->get_logger(), "Safety: /cmd_vel_real not available, suppressing CAN output");
        if (monitor_exists) {
            auto stats = velocity_monitor_->getStatistics();
            RCLCPP_INFO(this->get_logger(), "[DEBUG] VelocityMonitor stats: total_msgs=%lu, invalid_msgs=%lu, avg_freq=%.2f", stats.total_messages, stats.invalid_messages, stats.average_frequency);
            auto ms_since_last = velocity_monitor_->getTimeSinceLastMessage().count();
            RCLCPP_INFO(this->get_logger(), "[DEBUG] ms since last /cmd_vel_real: %ld", static_cast<long>(ms_since_last));
        }
        return;
    }
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    double left_wheel_velocity = linear_velocity - (angular_velocity * wheel_separation_ / 2.0);
    double right_wheel_velocity = linear_velocity + (angular_velocity * wheel_separation_ / 2.0);

    left_wheel_velocity = std::clamp(left_wheel_velocity, -max_velocity_, max_velocity_);
    right_wheel_velocity = std::clamp(right_wheel_velocity, -max_velocity_, max_velocity_);

    sendCanMessage(left_wheel_velocity, right_wheel_velocity);
}

void CmdVelToCANNode::sendCanMessage(double left_wheel_velocity, double right_wheel_velocity) {
    can_msgs::msg::Frame left_wheel_msg;
    can_msgs::msg::Frame right_wheel_msg;

    // Use correct VESC CAN IDs and set CAN_EFF_FLAG (from linux/can.h)
    left_wheel_msg.id = 0x1C | CAN_EFF_FLAG;  // 28 | extended
    right_wheel_msg.id = 0x2E | CAN_EFF_FLAG; // 46 | extended

    // Map velocity to duty cycle using empirical calibration (2025): duty_cycle = velocity_mps / 7.857
    // See DESIGN_AND_API.md for details
    double left_duty = std::clamp(left_wheel_velocity / 7.857, -1.0, 1.0);
    double right_duty = std::clamp(right_wheel_velocity / 7.857, -1.0, 1.0);

    int32_t left_val = static_cast<int32_t>(left_duty * 100000.0);
    int32_t right_val = static_cast<int32_t>(right_duty * 100000.0);

    // Zero the data arrays and set dlc
    left_wheel_msg.dlc = 4;
    right_wheel_msg.dlc = 4;
    for (int i = 0; i < 8; ++i) {
        left_wheel_msg.data[i] = 0;
        right_wheel_msg.data[i] = 0;
    }
    // Big-endian packing
    left_wheel_msg.data[0] = (left_val >> 24) & 0xFF;
    left_wheel_msg.data[1] = (left_val >> 16) & 0xFF;
    left_wheel_msg.data[2] = (left_val >> 8) & 0xFF;
    left_wheel_msg.data[3] = left_val & 0xFF;
    right_wheel_msg.data[0] = (right_val >> 24) & 0xFF;
    right_wheel_msg.data[1] = (right_val >> 16) & 0xFF;
    right_wheel_msg.data[2] = (right_val >> 8) & 0xFF;
    right_wheel_msg.data[3] = right_val & 0xFF;

    RCLCPP_INFO(this->get_logger(), "[CAN DEBUG] Left: id=0x%X dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X] | Right: id=0x%X dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
        left_wheel_msg.id, left_wheel_msg.dlc,
        left_wheel_msg.data[0], left_wheel_msg.data[1], left_wheel_msg.data[2], left_wheel_msg.data[3],
        left_wheel_msg.data[4], left_wheel_msg.data[5], left_wheel_msg.data[6], left_wheel_msg.data[7],
        right_wheel_msg.id, right_wheel_msg.dlc,
        right_wheel_msg.data[0], right_wheel_msg.data[1], right_wheel_msg.data[2], right_wheel_msg.data[3],
        right_wheel_msg.data[4], right_wheel_msg.data[5], right_wheel_msg.data[6], right_wheel_msg.data[7]
    );

    can_publisher_->publish(left_wheel_msg);
    can_publisher_->publish(right_wheel_msg);
}

} // namespace cmdvel2can
