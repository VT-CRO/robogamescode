import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'articubot_one'  # Ensure this matches your package name

    # === USE SIMULATED CAMERA ===
    gz_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        output="screen"
    )

    # === USE REALSENSE CAMERA (IF USING REAL HARDWARE) ===
    rs_camera_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    rs_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_camera_launch_file)
    )

    # === OPTIONAL: USE USB CAMERA (IF USING A USB CAMERA) ===
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical'
        }]
    )

    return LaunchDescription([
        gz_camera_bridge,  # Use this for simulation
        # rs_camera_launch,  # Uncomment if using a RealSense camera
        # v4l2_camera_node  # Uncomment if using a USB camera
    ])
