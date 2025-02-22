#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <cmath>

class MicroROSInterfaceNode : public rclcpp::Node
{
public:
    MicroROSInterfaceNode() : Node("micro_ros_bridge_interface_node")
    {
        // Subscriber: Receives joint angles from the kinematics node
        joint_angle_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10, 
            std::bind(&MicroROSInterfaceNode::jointAngleCallback, this, std::placeholders::_1));

        // Subscription: Feedback from the servos (Micro-ROS Teensy)
        servo_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/servo_feedback", 10, std::bind(&MicroROSInterfaceNode::servoFeedbackCallback, this, std::placeholders::_1));
            //binds the first parameter (1) of the servoFeedbackCallback function to the value (this)
            //->when called again the function will preserve the preset value for that param

        // Publisher: Sends processed servo commands to the Micro-ROS node (Teensy)
        servo_command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_commands", 10);
            
        RCLCPP_INFO(this->get_logger(), "Micro-ROS Interface Node Initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_command_pub_;

    std::vector<float> last_servo_commands_;

    std::vector<float> convertJointAnglesToServoCommands(const std::vector<double>& joint_angles)
    {
        std::vector<float> servo_commands;
        for (double angle : joint_angles)
        {
            // Map [-pi, pi] to [500, 2500] microseconds PWM
            float pwm = 1500 + (angle / M_PI) * 1000;
            servo_commands.push_back(pwm);
        }
        return servo_commands;
    }

    void jointAngleCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "JointAngleCallback fct called");

        // Convert joint angles to servo commands
        std::vector<float> servo_commands = convertJointAnglesToServoCommands(msg->position);

        // Publish servo commands
        std_msgs::msg::Float32MultiArray servo_msg;
        servo_msg.data = servo_commands;
        servo_command_pub_->publish(servo_msg);

        // Store last sent commands for comparison with feedback
        last_servo_commands_ = servo_commands;

        RCLCPP_INFO(this->get_logger(), "Published Servo Commands.");
    }

    // Callback: Receive feedback from Micro-ROS and compare with desired joint states
    void servoFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ServoFeedbackCallback fct called");

        if (msg->position.size() != last_servo_commands_.size())
        {
            RCLCPP_WARN(this->get_logger(), "Mismatch in feedback and sent commands size.");
            return;
        }

        for (size_t i = 0; i < msg->position.size(); ++i)
        {
            double error = std::abs(last_servo_commands_[i] - msg->position[i]);
            if (error > 50.0) // Example threshold for a significant deviation
            {
                RCLCPP_WARN(this->get_logger(), "Servo %lu has a large deviation: %.2f", i, error);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MicroROSInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
