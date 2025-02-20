#! /bin/bash

rm ./src/leg/leg1_description/urdf/leg1.urdf
rm ./src/leg/leg1_description/urdf/leg1.sdf
bash & source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro ./src/leg/leg1_description/urdf/leg1.xacro > ./src/leg/leg1_description/urdf/leg1.urdf
gz sdf -p ./src/leg/leg1_description/urdf/leg1.urdf > ./src/leg/leg1_description/urdf/leg1.sdf
exit

