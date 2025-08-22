from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("roboturdf", package_name="moveit_config")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .robot_description_kinematics(file_path="config/kinematics.yaml") # Add this line
    .to_moveit_configs())
    return generate_move_group_launch(moveit_config)
