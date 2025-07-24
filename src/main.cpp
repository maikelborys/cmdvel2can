/**
 * @file main.cpp
 * @brief Entry point for the CmdVel to CAN Bridge ROS2 node
 * 
 * This file contains the main function that initializes and runs
 * the CmdVel to CAN Bridge node.
 */

#include <rclcpp/rclcpp.hpp>
#include "cmdvel2can/cmdvel_to_can_bridge.hpp"

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        // Create and run the node
        auto node = std::make_shared<cmdvel2can::CmdVelToCANNode>();
        node->initialize();
        RCLCPP_INFO(node->get_logger(), "Starting CmdVel to CAN Bridge Node");
        // Spin the node
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
        return 1;
    }
    
    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
