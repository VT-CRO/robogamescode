#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>

class MoveGroupInterfaceDemo : public rclcpp::Node
{
public:
    MoveGroupInterfaceDemo() : Node("move_group_interface_demo")
    {
        // Constructor body can remain empty or contain any other initialization
    }

    void moveArm()
    {
        // Create MoveGroupInterface object for the "arm" group after the object is fully initialized
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");

        // Set a random target for the robot arm to move to
        move_group.setRandomTarget();

        // Move the robot arm to the random target
        move_group.move();

        //Plan motion without executing it
        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // auto error_code = move_group.plan(plan);

        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin the node
    auto node = std::make_shared<MoveGroupInterfaceDemo>();
    node->moveArm();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//Deprecated from ros1

//#include <moveit/move_group_interface/move_group_interface.h>
// //this is a ROS node. It initializes the ROS system, sets up the MoveGroupInterface (which is part of the MoveIt! API), and controls the robot arm based on a random target.

// int main(int argc, char **argv)
// {
//     //Creating a node called move_group_interface_demo
//     // . The AnonymousName option ensures that if multiple nodes with the same name are launched, they get unique names (important for launching multiple instances).
//     ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    
//     // Start a ROS spinning thread
//     //responsible for processing ROS callbacks (i.e., handling messages, services, etc.) while the program is running. 
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     //creates a MoveGroupInterface object for the "arm" group used to control the mvt of the robot
//     moveit::planning_interface::MoveGroupInterface group("arm");

//     //Set target as random for now
//     group.setRandomTarget();

//     //makes the robot arm move to the random target set earlier.
//     group.move();

//     ros::waitForShutdown();

// }