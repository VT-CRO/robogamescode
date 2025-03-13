import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='mini_car_bot' 

    #robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

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
        #remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'simple.world')

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description="World to load"
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args':[ '-r -v4 ', world], 'on exit shutdown': 'true'}.items()
             )
    #  ^^^
    #since world doesn't get passed transitvely like it did in previous gazebo versions
    #we now need to provide it ourselves by setting it up as a launch argument to this launch file
    #v4 feature to print logs to screen and space after v4 is important

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.

    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',  #create file provided in ros_gz_sim package
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-z', '0.1'], #spawn robot a little bit above the ground to prevent falling thru the gnd
        output='screen')
    
    #starts the controllers automatically (we can add delays so that each spawner waits for other spawner to finish before starting)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
    )

    #since the bridge params file isnt a ros params yaml file, we can't pass it straight into the
    #parameters argument like this parameters=[bridge_params]. Instead we use the regular parameter config_file which is the path to our bridge_params
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        #joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])