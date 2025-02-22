#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MoveGroupInterfaceNode : public rclcpp::Node
{
public:
    MoveGroupInterfaceNode() : Node("move_group_interface_tutorial")
    {
        // Create MoveGroupInterface
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        //Manages objects in the planning scene (e.g., obstacles, tables, or any other elements that the robot should avoid)
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Publisher publishes trajectory plans to the /move_group/display_planned_path topic for visualization in RViz.
        auto display_publisher = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/move_group/display_planned_path", 1);

        // Print reference frames
        RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End Effector link: %s", move_group->getEndEffectorLink().c_str());

        // Define target pose via desired position and orientation of the robot’s end-effector.
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.x = 0;
        target_pose1.orientation.y = 0;
        target_pose1.orientation.z = 0;
        target_pose1.orientation.w = 1;
        target_pose1.position.x = 1.0;
        target_pose1.position.y = 1.0;
        target_pose1.position.z = 1.0;
        move_group->setPoseTarget(target_pose1);

        // Plan motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Visualizing plan %s", success ? "SUCCESS" : "FAILED");

        // Execute motion if planning was successful
        if (success)
        {
            move_group->move();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//ROS 1 Deprecated

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "move_group_interface_tutorial");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     sleep(2.0);

//     moveit::planning_interface::MoveGroupInterface group("arm");
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//     moveit_msgs::DisplayTrajectory display_trajectory;

//     ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
//     ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

//     // Target position
//     geometry_msgs::Pose target_pose1;
//     target_pose1.orientation.x = 0;
//     target_pose1.orientation.y = 0;
//     target_pose1.orientation.z = 0;
//     target_pose1.orientation.w = 1;

//     target_pose1.position.x = 1;
//     target_pose1.position.y = 1;
//     target_pose1.position.z = 1;
//     group.setPoseTarget(target_pose1);

//     // visualize the planning
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
//     ROS_INFO("visualizeing plan %s", success.val ? "":"FAILED");

//     // move the group arm
//     group.move();

//     ros::shutdown();
//     return 0;

// }