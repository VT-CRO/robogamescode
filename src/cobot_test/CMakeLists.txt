cmake_minimum_required(VERSION 3.8)
project(cobot_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare the executable and its source file
add_executable(test_random_cobot_node src/test_random_cobot.cpp)
add_executable(test_custom_cobot_node src/test_custom_cobot.cpp)
#add_executable(checking_collision_node src/checking_collision.cpp)
# add_executable(robot_loader_with_kinematics_node src/robot_model_loader_and_kinematics.cpp)
add_executable(robot_kinematics src/robot_kinematics.cpp)
add_executable(micro_ros_bridge_interface src/micro_ros_bridge_interface.cpp)
add_executable(perception src/perception.cpp)


target_include_directories(
  robot_kinematics
  PUBLIC include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/opt/ros/jazzy/include)
include_directories(/opt/ros/jazzy/include/eigen_stl_containers)


ament_target_dependencies(
  robot_kinematics
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

ament_target_dependencies(
  micro_ros_bridge_interface
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

ament_target_dependencies(
  perception
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  cv_bridge
  image_transport
  OPENCV
)

# Link against the required libraries
ament_target_dependencies(
  test_random_cobot_node
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

# Link against the required libraries
ament_target_dependencies(
  test_custom_cobot_node
  rclcpp
  moveit_ros_planning_interface
  std_msgs
)

# # Link against the required libraries
# ament_target_dependencies(
#   checking_collision_node
#   rclcpp
#   moveit_ros_planning_interface
#   std_msgs
# )

# Install targets
install(TARGETS
  test_random_cobot_node
  test_custom_cobot_node
  robot_kinematics 
  micro_ros_bridge_interface
  perception
  #checking_collision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
