import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    share_dir = get_package_share_directory('leg1_description')

    # Process the Xacro file into URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'leg1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Publish the URDF to the /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Start Gazebo Sim (Harmonic)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim'],
        output='screen'
    )

    # Spawn the robot in Gazebo Sim
    spawn_entity = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/default/create_entity',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000',
             '--req', f'name: "leg1" sdf: "{robot_urdf}"'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
    ])
