#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <cstdio>
#include <termios.h>


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
        //servo_command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_commands", 10);
         // Open serial port (adjust the port name if needed)
        
         //serial_fd_ = open_serial("/dev/serial0", 115200);
        serial_fd_ = open_serial("/dev/ttyACM0", 115200);

        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        }

        RCLCPP_INFO(this->get_logger(), "Micro-ROS Bridge Interface Node Initialized");

        
        RCLCPP_INFO(this->get_logger(), "Micro-ROS Interface Node Initialized");
    }

    ~MicroROSInterfaceNode() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_feedback_sub_;
    //rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr servo_command_pub_;

    int serial_fd_{-1};

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
        // Store last sent commands for comparison with feedback
        last_servo_commands_ = servo_commands;

        // Publish servo commands
        // std_msgs::msg::Float32MultiArray servo_msg;
        // servo_msg.data = servo_commands;
        // servo_command_pub_->publish(servo_msg);
        // Build a comma-separated string (e.g., "1500,1700\n")
        std::ostringstream oss;
        for (size_t i = 0; i < servo_commands.size(); ++i)
        {
            oss << servo_commands[i];
            if (i < servo_commands.size() - 1)
                oss << ",";
        }
        oss << "\n";
        std::string command_str = oss.str();
        RCLCPP_INFO(this->get_logger(), "Sending servo command: %s", command_str.c_str());

        // Write the command to the serial port.
        if (serial_fd_ >= 0)
        {
            ssize_t bytes_written = write(serial_fd_, command_str.c_str(), command_str.length());
            if (bytes_written < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
            } else {
                //tcflush(serial_fd_, TCIOFLUSH); // 🛠 Flush the output buffer
                RCLCPP_INFO(this->get_logger(), "Flushed Serial Port");
            }
        }
        

        //RCLCPP_INFO(this->get_logger(), "Published Servo Commands.");
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

     // Helper function to open and configure a serial port.
     int open_serial(const char* port, int baud)
     {
        int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            perror("open_serial: Unable to open port");
            return -1;
        }
        // Set file descriptor to blocking mode.
        // fcntl(fd, F_SETFL, 0);

        struct termios options;
        tcgetattr(fd, &options);

        // Set baud rate.
        speed_t brate;
        switch(baud)
        {
            case 9600:   brate = B9600;   break;
            case 19200:  brate = B19200;  break;
            case 38400:  brate = B38400;  break;
            case 57600:  brate = B57600;  break;
            case 115200: brate = B115200; break;
            default:     brate = B115200; break;
        }
        cfsetispeed(&options, brate);
        cfsetospeed(&options, brate);

        // Configure 8N1 (8 data bits, no parity, 1 stop bit)
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // Set raw input and output mode.
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        tcsetattr(fd, TCSANOW, &options);
        return fd;
     }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MicroROSInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
