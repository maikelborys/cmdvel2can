// SocketCAN implementation for VESC communication
// Supports verified VESC CAN protocol with IDs 28 (0x1C) and 46 (0x2E)

#include "cmdvel2can/can_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>

namespace cmdvel2can {

// TODO: Implement CANInterface class
// This file will contain:
// - SocketCAN initialization and configuration
// - Thread-safe CAN frame transmission
// - Error handling and recovery
// - Connection monitoring

CANInterface::CANInterface(const std::string& interface_name)
    : interface_name_(interface_name), socket_fd_(-1), is_initialized_(false) {
    // Implementation needed
}

CANInterface::~CANInterface() {
    shutdown();
}

bool CANInterface::initialize() {
    RCLCPP_INFO(rclcpp::get_logger("can_interface"), "Initializing CAN interface: %s", interface_name_.c_str());
    
    // Create CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("can_interface"), "Failed to create CAN socket: %s", strerror(errno));
        return false;
    }
    
    // Find the CAN interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("can_interface"), "Failed to get interface index for %s: %s", 
                     interface_name_.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Bind socket to CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("can_interface"), "Failed to bind to CAN interface %s: %s", 
                     interface_name_.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    is_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("can_interface"), "CAN interface %s initialized successfully", interface_name_.c_str());
    return true;
}

bool CANInterface::sendFrame(const can_frame& frame) {
    if (!is_initialized_ || socket_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("can_interface"), "CAN interface not initialized");
        return false;
    }
    
    // Use exact same approach as working test - direct write without extra copying
    ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("can_interface"), 
                     "Failed to send CAN frame (sent %zd bytes, expected %zu): %s", 
                     bytes_sent, sizeof(frame), strerror(errno));
        return false;
    }
    
    // Log the sent frame for debugging
    RCLCPP_INFO(rclcpp::get_logger("can_interface"), 
                 "Sent CAN frame: ID=0x%08X, DLC=%d, Data=%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X",
                 frame.can_id, frame.can_dlc,
                 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    
    return true;
}

bool CANInterface::receiveFrame(can_frame& frame) {
    // Implementation needed
    return false;
}

bool CANInterface::isConnected() const {
    return is_initialized_ && socket_fd_ >= 0;
}

void CANInterface::shutdown() {
    if (socket_fd_ >= 0) {
        RCLCPP_INFO(rclcpp::get_logger("can_interface"), "Shutting down CAN interface");
        close(socket_fd_);
        socket_fd_ = -1;
    }
    is_initialized_ = false;
}

} // namespace cmdvel2can
