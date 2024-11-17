import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_robot_description(package_name, urdf_filename):
    # Locate the package
    package_path = get_package_share_directory(package_name)
    # Locate the URDF file within the package
    urdf_path = os.path.join(package_path, 'urdf', urdf_filename)

    # Check if the file is a Xacro file and process it
    if urdf_filename.endswith('.xacro'):
        # Use xacro to process the file
        doc = xacro.process_file(urdf_path)
        # Convert the xacro document to XML string
        robot_description = doc.toprettyxml(indent='  ')
    else:
        # Read the URDF file directly
        with open(urdf_path, 'r') as file:
            robot_description = file.read()

    # # Print the robot description for debugging
    # print("Robot Description:\n", robot_description)

    # Return the robot description parameter
    return {'robot_description': robot_description}


def generate_launch_description():
    # Get the path to the 'gazebo_ros' package and the world file
    gazebo_pkg_name = 'ros_gz'
    gazebo_package = FindPackageShare(gazebo_pkg_name)
    world_file = os.path.join(gazebo_package.find(gazebo_pkg_name), 'worlds', 'empty.world')  # Adjust if using a custom world

    # Get the path to your robot's URDF file
    urdf_file = os.path.join(FindPackageShare('first_package').find('first_package'), 'urdf', 'robot_model.urdf')

    return LaunchDescription([
        # Launch Gazebo
        Node(
            package= gazebo_pkg_name,  # Gazebo ROS package
            executable='gazebo',  # Gazebo executable
            name='gazebo',
            output='screen',
            arguments=[world_file],  # The Gazebo world file
        ),
        
        # Spawn robot in Gazebo
        Node(
            package= gazebo_pkg_name,  # Gazebo ROS package
            executable='spawn_entity.py',  # Spawn robot script
            name='spawn_entity',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot'],  # Use robot_description parameter
        ),
    ])



# <launch>
#   <include
#     file="$(find gazebo_ros)/launch/empty_world.launch" />
#   <node
#     name="tf_footprint_base"
#     pkg="tf"
#     type="static_transform_publisher"
#     args="0 0 0 0 0 0 base_link base_footprint 40" />
#   <node
#     name="spawn_model"
#     pkg="gazebo_ros"
#     type="spawn_model"
#     args="-file $(find go2_description)/urdf/go2_description.urdf -urdf -model go2_description"
#     output="screen" />
#   <node
#     name="fake_joint_calibration"
#     pkg="rostopic"
#     type="rostopic"
#     args="pub /calibrated std_msgs/Bool true" />
# </launch>