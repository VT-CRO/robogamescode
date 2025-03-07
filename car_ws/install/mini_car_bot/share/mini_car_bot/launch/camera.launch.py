import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
        rs_camera_feedback_launch,
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     output='screen',
        #     parameters=[{
        #         'image_size': [640,480],
        #         'camera_frame_id': 'camera_link_optical'
        #         }]
        # )
    ])