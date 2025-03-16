//This node listens for the detected cone and sends navigation goals accordingly.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ConeNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ConeNavigator() : Node("cone_navigator")
    {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        cone_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detected_cone", 10, std::bind(&ConeNavigator::process_cone, this, std::placeholders::_1));

        while (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "Waiting for 'navigate_to_pose' action server...");
        }
    }

private:
    void process_cone(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string detected_cone = msg->data;
        RCLCPP_INFO(get_logger(), "Detected cone: %s", detected_cone.c_str());

        geometry_msgs::msg::PoseStamped goal = generate_goal(detected_cone);
        send_goal(goal);
    }

    geometry_msgs::msg::PoseStamped generate_goal(const std::string &cone_color)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";

        if (cone_color == "yellow")
            goal.pose.position.y += 1.0;  // Turn left
        else if (cone_color == "blue")
            goal.pose.position.y -= 1.0;  // Turn right
        else if (cone_color == "red")
            goal.pose.position.x += 1.0;  // Move between red cones
        else if (cone_color == "ramp")
            goal.pose.position.z += 0.5;  // Climb ramp

        goal.pose.orientation.w = 1.0;
        return goal;
    }

    void send_goal(const geometry_msgs::msg::PoseStamped &goal)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cone_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeNavigator>());
    rclcpp::shutdown();
    return 0;
}
