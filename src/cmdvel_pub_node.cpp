#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>
#include <string>

class CmdvelPubNode : public rclcpp::Node {
public:
    CmdvelPubNode() : Node("cmdvel_pub_node") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "cmdvel_pub_node started. Type 'x <value>' and press Enter to send linear.x, or 'stop' to send zero.");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdvelPubNode::check_stdin, this));
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void check_stdin() {
        static std::string last_line;
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(0, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        int rv = select(1, &set, NULL, NULL, &timeout);
        if(rv > 0) {
            std::string line;
            std::getline(std::cin, line);
            if (!line.empty() && line != last_line) {
                last_line = line;
                geometry_msgs::msg::Twist msg;
                if (line == "stop") {
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                } else if (line.rfind("x ", 0) == 0) {
                    try {
                        msg.linear.x = std::stod(line.substr(2));
                        msg.angular.z = 0.0;
                    } catch (...) {
                        RCLCPP_WARN(this->get_logger(), "Invalid input. Use 'x <value>' or 'stop'.");
                        return;
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid input. Use 'x <value>' or 'stop'.");
                    return;
                }
                pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published: linear.x=%.3f angular.z=%.3f", msg.linear.x, msg.angular.z);
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdvelPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
