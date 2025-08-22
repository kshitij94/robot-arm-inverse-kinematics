from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder # Add this import

def generate_launch_description():
    moveit_config = ( # Add this block
        MoveItConfigsBuilder("roboturdf", package_name="moveit_config")
        .robot_description(
            file_path="config/roboturdf.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/roboturdf.srdf") # Assuming this is needed
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='hello_moveit',
            executable='hello_moveit',
            name='hello_moveit',
            output='screen',
            parameters=[ # Add parameters
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])