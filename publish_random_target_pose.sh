#!/bin/bash

while true; do
    x=$(awk -v min=-1.0 -v max=1.0 'BEGIN{srand(); print min+rand()*(max-min)}')
    y=$(awk -v min=-1.0 -v max=1.0 'BEGIN{srand(); print min+rand()*(max-min)}')
    z=$(awk -v min=0.1 -v max=1.0 'BEGIN{srand(); print min+rand()*(max-min)}')

    ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
        header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
        pose: {position: {x: $x, y: $y, z: $z},
               orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
    }"

    sleep 1  # Adjust interval as needed
done
