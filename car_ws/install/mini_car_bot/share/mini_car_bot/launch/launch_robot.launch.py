import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='mini_car_bot' 

    #robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    #use sim time has to be false in real life

    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name),'launch','joystick.launch.py'
    #     )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    #since we're not working with gazebo here, we need to provided the robot_description topic to the controller manager ourselves
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    urdf_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Then pass it as a parameter:
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_desc}, controller_params_file],
    )


    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     arguments=[{'robot_description': robot_description}, controller_params_file],
    # )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action = controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action = controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        #joystick,
        twist_mux,
        delayed_diff_drive_spawner,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
    ])