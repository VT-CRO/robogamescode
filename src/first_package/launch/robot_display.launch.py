import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPAckageShare(__package__='first_package').find('first_package')
    urdfModelPath = os.path.join(pkgPath, 'urdf/robot.urdf')
    rvizConfigPath = os.path.join(pkgPath, 'config/config.rviz') #doesnt matter that much

    #print path just for debugging
    print(urdfModelPath)

    #store the robot_description from urdf file to the robot_desc dictionary
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}

    #define the first node state publisher for the full robot
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[urdfModelPath]
    )

    #define the second node state publisher for the robot joints
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdfModelPath]
    )

    #define the third node state publisher for the gui of the robot joints (slider for the joints)
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdfModelPath],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    #define the fourth node state publisher for the full robot
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rivz2',
        output = 'screen',
        arguments=['-d', rvizConfigPath]    #rviz will search for the file to print to from that path
    )

    #Initialize the Launch Description, which starts the gui joints by default
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='This is a flag for joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

    