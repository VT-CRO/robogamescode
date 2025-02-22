#! /bin/bash

# This script is made to run rviz without needing to rebuild all the time
#

#bash & rm -rf build/ install/ log/
bash & source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ws_moveit2/install/setup.bash
source install/setup.bash
ros2 launch panda_robot_moveit_config demo.launch.py

#exit


