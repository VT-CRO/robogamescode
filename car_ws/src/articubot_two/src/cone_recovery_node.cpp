#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RecoveryNode : public rclcpp::Node {
public:
    RecoveryNode() : Node("recovery_node") {
        // Publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber to detected objects
        object_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detected_cone", 10, std::bind(&RecoveryNode::object_callback, this, std::placeholders::_1));

        // Timer to check for recovery conditions
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&RecoveryNode::check_recovery, this));
    }

private:
    void object_callback(const std_msgs::msg::String::SharedPtr msg) {
        detected_object_ = msg->data;
    }

    void check_recovery() {
        if (detected_object_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No cone detected! Rotating to find one...");
            rotate_in_place();
        }
    }

    void rotate_in_place() {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.5;  // Slow rotation
        cmd_pub_->publish(cmd);
        rclcpp::sleep_for(std::chrono::seconds(2));
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string detected_object_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecoveryNode>());
    rclcpp::shutdown();
    return 0;
}
