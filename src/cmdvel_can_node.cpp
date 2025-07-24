
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <can_msgs/msg/frame.hpp>
#include <cstring>
#include <string>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif


class Cmdvel2CanNode : public rclcpp::Node {
public:
    Cmdvel2CanNode(const rclcpp::NodeOptions& options) : Node("cmdvel2can", options) {
        // Parameters
        wheel_separation_ = this->declare_parameter("wheel_separation", 0.370);
        wheel_diameter_ = this->declare_parameter("wheel_diameter", 0.356);
        left_vesc_id_ = this->declare_parameter("left_vesc_id", 28);
        right_vesc_id_ = this->declare_parameter("right_vesc_id", 46);
        max_velocity_ = this->declare_parameter("max_velocity", 2.0);
        max_angular_velocity_ = this->declare_parameter("max_angular_velocity", 3.0);
        command_frequency_ = this->declare_parameter("command_frequency", 50.0);
        watchdog_timeout_ = this->declare_parameter("watchdog_timeout", 0.5);

        // ROS publishers/subscribers
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("/cmdvel2can_debug", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                last_cmd_vel_ = *msg;
                last_cmdvel_time_ = this->get_clock()->now();
                /******************TEMPORARY *******************************************
                every time a new /cmd_vel message arrives, your node will instantly compute and publish CAN frames to the can_tx topic (in addition to the periodic timer-based publishing).
                This gives you both low-latency response and robust periodic safety updates. The bridge node is still required to actually send frames to the CAN bus.
             
                // Immediate CAN publish for low-latency response
                auto [left_vel, right_vel] = cmdvel_to_wheel_velocities(msg->linear.x, msg->angular.z, wheel_separation_);
                left_vel = std::clamp(left_vel, -max_velocity_, max_velocity_);
                right_vel = std::clamp(right_vel, -max_velocity_, max_velocity_);
                double left_duty = velocity_to_duty_cycle(left_vel, max_velocity_);
                double right_duty = velocity_to_duty_cycle(right_vel, max_velocity_);
                can_pub_->publish(build_duty_cycle_frame(left_vesc_id_, left_duty));
                can_pub_->publish(build_duty_cycle_frame(right_vesc_id_, right_duty));
                // Optional: debug message
                std_msgs::msg::String dbg;
                dbg.data = "[IMMEDIATE] Sent: left_duty=" + std::to_string(left_duty) + ", right_duty=" + std::to_string(right_duty);
                debug_pub_->publish(dbg);
                   */
            }
        );
        cmd_vel_real_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_real", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr) {
                last_cmdvel_real_time_ = this->get_clock()->now();
            }
        );

        // Timer for control loop
        auto period = std::chrono::duration<double>(1.0 / command_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Cmdvel2CanNode::control_loop, this)
        );
    }

private:
    // Parameters
    double wheel_separation_, wheel_diameter_;
    int left_vesc_id_, right_vesc_id_;
    double max_velocity_, max_angular_velocity_;
    double command_frequency_, watchdog_timeout_;

    // ROS
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_real_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    geometry_msgs::msg::Twist last_cmd_vel_;
    rclcpp::Time last_cmdvel_time_;
    rclcpp::Time last_cmdvel_real_time_;

    // Main control loop
    void control_loop() {
        // Safety: only run if /cmd_vel_real is alive
        auto now = this->get_clock()->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(watchdog_timeout_);
        if ((now - last_cmdvel_real_time_) > timeout) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "/cmd_vel_real is stale, not sending CAN");
            return;
        }
        // Convert cmd_vel to wheel velocities
        double lin = last_cmd_vel_.linear.x;
        double ang = last_cmd_vel_.angular.z;
        auto [left_vel, right_vel] = cmdvel_to_wheel_velocities(lin, ang, wheel_separation_);
        // Clamp velocities
        left_vel = std::clamp(left_vel, -max_velocity_, max_velocity_);
        right_vel = std::clamp(right_vel, -max_velocity_, max_velocity_);
        // Convert to duty cycle
        double left_duty = velocity_to_duty_cycle(left_vel, max_velocity_);
        double right_duty = velocity_to_duty_cycle(right_vel, max_velocity_);
        // Build and publish CAN frames
        can_pub_->publish(build_duty_cycle_frame(left_vesc_id_, left_duty));
        can_pub_->publish(build_duty_cycle_frame(right_vesc_id_, right_duty));
        // Debug
        std_msgs::msg::String dbg;
        dbg.data = "Sent: left_duty=" + std::to_string(left_duty) + ", right_duty=" + std::to_string(right_duty);
        debug_pub_->publish(dbg);
    }

    std::pair<double, double> cmdvel_to_wheel_velocities(double lin, double ang, double sep) {
        double left = lin - (ang * sep / 2.0);
        double right = lin + (ang * sep / 2.0);
        return {left, right};
    }
    // Empirical calibration (2025): duty_cycle = velocity_mps / 7.857
    double velocity_to_duty_cycle(double v, double /*vmax*/) {
        // See DESIGN_AND_API.md for details
        return std::clamp(v / 7.857, -1.0, 1.0);
    }
    can_msgs::msg::Frame build_duty_cycle_frame(int vesc_id, double duty) {
        can_msgs::msg::Frame frame;
        frame.id = static_cast<uint32_t>(vesc_id) | CAN_EFF_FLAG;
        frame.dlc = 4;
        int32_t scaled = static_cast<int32_t>(duty * 100000);
        frame.data[0] = (scaled >> 24) & 0xFF;
        frame.data[1] = (scaled >> 16) & 0xFF;
        frame.data[2] = (scaled >> 8) & 0xFF;
        frame.data[3] = scaled & 0xFF;
        for (int i = 4; i < 8; ++i) frame.data[i] = 0;
        return frame;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    try {
        auto node = std::make_shared<Cmdvel2CanNode>(options);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}