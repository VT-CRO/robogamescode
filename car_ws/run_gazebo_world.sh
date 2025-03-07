#! /bin/bash

rm -rf build/ install/ log/
bash & source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch mini_car_bot launch_sim.launch.py
exit
