from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    urdf_path = "panda.urdf.xacro"
    srdf_path = "panda.srdf"

    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_robot_moveit_config").to_moveit_configs()
    # moveit_config.robot_description = urdf_path
    # moveit_config.robot_description_semantic = srdf_path
    return generate_demo_launch(moveit_config)
