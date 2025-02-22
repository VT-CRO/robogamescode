#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        // Subscribe to the RealSense RGB image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1)
        );

        // Publish detected obstacles
        obstacles_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Perception node initialized. Waiting for images...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacles_pub_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image to OpenCV format
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Option 1: Call Python Script for Object Detection (Recommended)
            std::string detected_objects = run_object_detection_script(frame);

            // Option 2: Do Object Detection in C++ (Optional)
            // std::string detected_objects = detect_objects_in_cpp(frame);

            // Publish detected objects
            auto obstacle_msg = std_msgs::msg::String();
            obstacle_msg.data = detected_objects;
            obstacles_pub_->publish(obstacle_msg);

        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    // **OPTION 1: Call Python Script for Object Detection**
    std::string run_object_detection_script(cv::Mat &frame) {
        // Save frame as a temporary file
        std::string temp_filename = "/tmp/temp_image.jpg";
        cv::imwrite(temp_filename, frame);

        // Run Python script (replace with actual script path)
        std::string command = "python3 /path/to/object_detection.py " + temp_filename;
        std::string result = exec(command.c_str());

        return result.empty() ? "No objects detected" : result;
    }

    // Function to execute shell command and return output
    std::string exec(const char *cmd) {
        char buffer[128];
        std::string result;
        FILE *pipe = popen(cmd, "r");
        if (!pipe) return "ERROR";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        return result;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
