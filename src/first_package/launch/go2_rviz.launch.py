import os
from launch import LaunchDescription
from launch_ros.actions import Node
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
    # Generate the robot description from the URDF file
    robot_description_param = generate_robot_description('first_package', 'unitree_go2_description.urdf')

    return LaunchDescription([
        # Declare a launch argument for user_debug (if needed)
        DeclareLaunchArgument(
            name='user_debug',
            default_value='false',
            description='Enable or disable user debug'
        ),
        
        # Node for joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}]
        ),
        #This is essentially the same as joint_state_publisher, but it comes with a graphical user interface (GUI)
        
        # Node for robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description_param],
            output='screen'
        ),
        
        # Node for RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', os.path.join(
            #     get_package_share_directory('first_package'), 'launch', 'check_joint.rviz')],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

#previous code for .launch file
# <launch>

#     <arg name="user_debug" default="false"/>
    
#     <param name="robot_description"  textfile="$(find go2_description)/urdf/go2_description.urdf" />

#     <!-- for higher robot_state_publisher average rate-->
#     <!-- <param name="rate" value="1000"/> -->

#     <!-- send fake joint values -->
#     <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui">
#         <param name="use_gui" value="TRUE"/>
#     </node>

#     <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
#         <param name="publish_frequency" type="double" value="1000.0"/>
#     </node>

#     <node pkg="rviz" type="rviz" name="rviz" respawn="false" output="screen"
#         args="-d $(find go2_description)/launch/check_joint.rviz"/>

# </launch>