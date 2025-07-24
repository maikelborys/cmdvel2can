#include "cmdvel2can/velocity_monitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

using namespace cmdvel2can;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("velocity_monitor_test");

    VelocityMonitor monitor(node);
    monitor.initialize("cmd_vel_real", 1000.0, 1.0); // 1s timeout, 1Hz min freq

    rclcpp::Rate rate(2.0); // 2 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        bool available = monitor.isDataAvailable();
        auto vel = monitor.getCurrentVelocity();
        RCLCPP_INFO(node->get_logger(), "cmd_vel_real available: %s | linear: %.3f angular: %.3f valid: %d",
            available ? "YES" : "NO", vel.linear, vel.angular, vel.is_valid);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
