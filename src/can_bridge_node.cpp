#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif

class CanBridgeNode : public rclcpp::Node {
public:
    CanBridgeNode(const rclcpp::NodeOptions& options) : Node("can_bridge", options) {
        can_interface_ = this->declare_parameter("can_interface", "can0");
        if (!open_can_socket(can_interface_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN interface %s", can_interface_.c_str());
            throw std::runtime_error("CAN init failed");
        }
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "can_tx", 100,
            std::bind(&CanBridgeNode::can_tx_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "CAN bridge node started on interface: %s", can_interface_.c_str());
    }
    ~CanBridgeNode() {
        if (can_sock_ >= 0) close(can_sock_);
    }
private:
    std::string can_interface_;
    int can_sock_ = -1;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    bool open_can_socket(const std::string& iface) {
        can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sock_ < 0) return false;
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ-1);
        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) return false;
        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) return false;
        return true;
    }
    void can_tx_callback(const can_msgs::msg::Frame::SharedPtr msg) {
        /* *********************************************************************************
        Timestamps and debug output have been added to your bridge node. Now, when a CAN frame is received on can_tx, the timestamp will be logged.
        */
        // Timestamp when CAN frame received from ROS2
        auto t_ros = std::chrono::system_clock::now();
        auto t_ros_us = std::chrono::time_point_cast<std::chrono::microseconds>(t_ros);
        auto epoch = t_ros_us.time_since_epoch();
        double t_ros_sec = epoch.count() / 1e6;
        RCLCPP_INFO(this->get_logger(), "[BRIDGE] Received can_tx at t=%.6f id=0x%X data=[%d,%d,%d,%d,%d,%d,%d,%d]", t_ros_sec, msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = msg->id;
        frame.can_dlc = msg->dlc;
        for (int i = 0; i < 8; ++i) frame.data[i] = msg->data[i];

        // Timestamp before write
        auto t_write = std::chrono::system_clock::now();
        auto t_write_us = std::chrono::time_point_cast<std::chrono::microseconds>(t_write);
        auto epoch_write = t_write_us.time_since_epoch();
        double t_write_sec = epoch_write.count() / 1e6;

        int n = write(can_sock_, &frame, sizeof(frame));
        if (n != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame (id=0x%X)", frame.can_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "[BRIDGE] Wrote CAN frame at t=%.6f id=0x%X", t_write_sec, frame.can_id);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    try {
        auto node = std::make_shared<CanBridgeNode>(options);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Fatal: %s\n", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
