// #include <rclcpp/rclcpp.hpp>
// #include <moveit/robot_model/robot_model.hpp>
// #include <moveit/robot_state/robot_state.hpp>
// #include <moveit/robot_model_loader/robot_model_loader.hpp>
// #include <moveit/planning_scene/planning_scene.hpp>
// #include <moveit/kinematic_constraints/utils.hpp>
// #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("arm_kinematics");
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread spinner([&executor]() { executor.spin(); });

//     // Load Robot Model
//     robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
//     //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//     const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//     planning_scene::PlanningScene planning_scene(kinematic_model);
//     RCLCPP_INFO(node->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//     // Collision Checking
//     collision_detection::CollisionRequest collision_request;
//     collision_detection::CollisionResult collision_result;
//     planning_scene.checkSelfCollision(collision_request, collision_result);

//     RCLCPP_INFO(node->get_logger(), "Test 1. Self collision Test: %s",
//                 collision_result.collision ? "in" : "not in");

//     // Get the current state of the robot
    
//     robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
//     std::vector<double> joint_values = {0, 0, 3.14, 0, 0, 0};
//     const moveit::core::JointModelGroup *joint_model_group = current_state.getJointModelGroup("arm");
//     current_state.setJointGroupPositions(joint_model_group, joint_values);

//     RCLCPP_INFO(node->get_logger(), "Test 2: Current state is %s",
//                 current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid");

//     // Detect collisions
//     collision_request.contacts = true;
//     collision_request.max_contacts = 1000;
//     collision_result.clear();
//     planning_scene.checkSelfCollision(collision_request, collision_result);

//     RCLCPP_INFO(node->get_logger(), "Test 3. Self collision Test: %s",
//                 collision_result.collision ? "in" : "not in");

//     for (const auto &contact : collision_result.contacts)
//     {
//         RCLCPP_INFO(node->get_logger(), "Test 4 . Contact between: %s and %s",
//                     contact.first.first.c_str(), contact.first.second.c_str());
//     }

//     // Modify Allowed Collision Matrix
//     collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
//     robot_state::RobotState copied_state = planning_scene.getCurrentState();

//     for (const auto &contact : collision_result.contacts)
//     {
//         acm.setEntry(contact.first.first, contact.first.second, true);
//     }
//     collision_result.clear();
//     planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

//     RCLCPP_INFO(node->get_logger(), "Test 5. Self collision after modified ACM: %s",
//                 collision_result.collision ? "in" : "not in");

//     // Add an object to the scene
//     moveit::planning_interface::PlanningSceneInterface current_scene;
//     rclcpp::sleep_for(std::chrono::seconds(5));

//     moveit_msgs::msg::CollisionObject cylinder;
//     cylinder.id = "cobot_cylinder";

//     shape_msgs::msg::SolidPrimitive primitive;
//     primitive.type = primitive.CYLINDER;
//     primitive.dimensions.resize(2);
//     primitive.dimensions[0] = 0.6; // Height
//     primitive.dimensions[1] = 0.2; // Radius

//     geometry_msgs::msg::Pose pose;
//     pose.orientation.w = 1.0;
//     pose.position.x = 0.0;
//     pose.position.y = 0.0;
//     pose.position.z = 0.4;

//     cylinder.primitives.push_back(primitive);
//     cylinder.primitive_poses.push_back(pose);
//     cylinder.operation = moveit_msgs::msg::CollisionObject::ADD;
//     cylinder.header.frame_id = "base_link";
    
//     std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//     collision_objects.push_back(cylinder);
//     current_scene.addCollisionObjects(collision_objects);

//     collision_detection::AllowedCollisionMatrix acm2 = planning_scene.getAllowedCollisionMatrix();
//     planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm2);

//     RCLCPP_INFO(node->get_logger(), "Test 6. Full collision test: %s",
//                 collision_result.collision ? "in" : "not in");

//     rclcpp::shutdown();
//     spinner.join();
//     return 0;
// }



// //ROS 1 Deprecated

// #include <ros/ros.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_scene/planning_scene.h>

// #include <moveit/kinematic_constraints/utils.h>
// #include <eigen_conversions/eigen_msg.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>


// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "arm_kinematics");

//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//     robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//     planning_scene::PlanningScene planning_scene(kinematic_model);

//     // Collision Checking
//     collision_detection::CollisionRequest collision_request;
//     collision_detection::CollisionResult collision_result;
//     planning_scene.checkSelfCollision(collision_request, collision_result);

//     ROS_INFO_STREAM("Test 1. Self collision Test: "<< (collision_result.collision ? "in" : "not in") << " self collision");



//     // Get the current state of the robot
//     robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

//     std::vector<double> joint_values ={0, 0 , 3.14, 0, 0, 0};
//     const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
//     current_state.setJointGroupPositions(joint_model_group, joint_values);

//     ROS_INFO_STREAM("Test 2: Current state is " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
    
    
//     // Information on the links that are colliding between each others
//     collision_request.contacts = true;
//     collision_request.max_contacts = 1000;

//     collision_result.clear();

//     planning_scene.checkSelfCollision(collision_request, collision_result);

//     ROS_INFO_STREAM("Test 3. Self collision Test: "<< (collision_result.collision ? "in" : "not in") << " self collision");

//     collision_detection::CollisionResult::ContactMap::const_iterator it;
//     for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
//     {
//         ROS_INFO("Test 4 . Contact between: %s and %s",
//                 it->first.first.c_str(),
//                 it->first.second.c_str());
//     }

//     // Modifying the Allowed Collision Matrix (ACM)

//     collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
//     robot_state::RobotState copied_state = planning_scene.getCurrentState();

//     collision_detection::CollisionResult::ContactMap::const_iterator it2;
//     for(it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
//     {
//         acm.setEntry(it2->first.first, it2->first.second, true);
//     }
//     collision_result.clear();

//     planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

//     ROS_INFO_STREAM("Test 5. Self collision after modified ACM: " << (collision_result.collision ? "in" : "not in")<< " self collision");

//     // Full collision checking

//     collision_result.clear();

//         // Add an object to the scene

//     moveit::planning_interface::PlanningSceneInterface current_scene;
//     sleep(5);

//     moveit_msgs::CollisionObject cylinder;
//     cylinder.id = "cobot_cylinder";

//     shape_msgs::SolidPrimitive primitive;
//     primitive.type = primitive.CYLINDER;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[0] = 0.6;
//     primitive.dimensions[1] = 0.2;
//     primitive.dimensions[2] = 0.2; 

//     geometry_msgs::Pose pose;
//     pose.orientation.w = 1.0;
//     pose.position.x = 0.0;
//     pose.position.y = 0.0;
//     pose.position.z = 0.4;

//     cylinder.primitives.push_back(primitive);
//     cylinder.primitive_poses.push_back(pose);
//     cylinder.operation = cylinder.ADD;
//         cylinder.header.frame_id = "base_link";
//     std::vector<moveit_msgs::CollisionObject> collision_objects;
//     collision_objects.push_back(cylinder);

//     current_scene.addCollisionObjects(collision_objects);



//     collision_detection::AllowedCollisionMatrix acm2 = planning_scene.getAllowedCollisionMatrix();

//     planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm2);

//     ROS_INFO_STREAM("Test 6. Full collision test: " << (collision_result.collision ? "in" : "not in")<< " collision");














//     ros::shutdown();
//     return 0;


// }