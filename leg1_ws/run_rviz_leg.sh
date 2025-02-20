#! /bin/bash

rm -rf build build/ install/ log/ 
bash & source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch leg1_description display.launch.py
exit
