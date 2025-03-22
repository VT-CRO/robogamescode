#! /bin/bash

bash & source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
exit
