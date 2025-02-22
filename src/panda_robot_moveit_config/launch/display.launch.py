import launch
import launch_ros.actions
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate and process the URDF (Xacro) file
    urdf_file = os.path.join(get_package_share_directory("panda_robot_moveit_config"), "config","panda.urdf.xacro",)

    robot_description = {"robot_description": xacro.process_file(urdf_file).toxml()}


    rsp_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    srdf_file = os.path.join(get_package_share_directory("panda_robot_moveit_config"), "config", "panda.srdf.yaml")

    kinematics_yaml = os.path.join(get_package_share_directory("panda_robot_moveit_config"), "config", "kinematics.yaml")

    move_group_node = launch_ros.actions.Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[{
            "robot_description_kinematics": kinematics_yaml,
            "robot_description_semantic": srdf_file,
        }],
        output="screen",
    )
    
    return launch.LaunchDescription([
        # Start the rsp
        rsp_node,

        # Start the controller manager
         # Delay controller_manager until robot_description is published
        launch.actions.TimerAction(
            period=5.0,  # Wait 5 seconds before launching controller_manager
            actions=[
                launch_ros.actions.Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=["/home/vtcro/panda_robot_ws/src/panda_robot_moveit_config/config/ros2_controllers.yaml"],
                    output="screen",
                ),
            ],
        ),

        move_group_node,

        # # Wait for controller_manager service to be available
        # launch.actions.TimerAction(
        #     period=10.0,
        #     actions=[
        #         # Load the arm controller
        #         launch_ros.actions.Node(
        #             package="controller_manager",
        #             executable="spawner",
        #             arguments=["arm_controller"],
        #             output="screen",
        #         ),
        #         # Load the gripper controller
        #         launch_ros.actions.Node(
        #             package="controller_manager",
        #             executable="spawner",
        #             arguments=["gripper_controller"],
        #             output="screen",
        #         ),
        #     ],
        # ),
    ])

