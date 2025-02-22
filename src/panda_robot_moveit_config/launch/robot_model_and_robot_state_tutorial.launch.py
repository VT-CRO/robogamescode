from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_robot_moveit_config").to_moveit_configs()

    print(moveit_config.robot_description_kinematics)

    robot_kinematics_node = Node(
        package="cobot_test",
        executable="robot_kinematics",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    micro_ros_interface_node = Node(
        package="cobot_test",
        executable="micro_ros_bridge_interface",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Path to the first launch file
    rs_camera_feedback_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # Include the first launch file
    rs_camera_feedback_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_camera_feedback_launch_file)
    )

    return LaunchDescription([
        robot_kinematics_node,
        micro_ros_interface_node,
        #rs_camera_feedback_launch
    ])