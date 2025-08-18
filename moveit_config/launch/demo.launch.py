import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launches import generate_rsp_launch,generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("roboturdf", package_name="moveit_config")
        .robot_description(
            file_path="config/roboturdf.urdf.xacro",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_config") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
# [rviz2-1] [WARN] [1755411046.621380456] [rviz2.moveit.ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    # Ros2 Control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )

    # Spawn controllers
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
            output="screen",
        )
        for controller in [
            "joint_state_broadcaster",
            "arm_controller",
        ]
    ]
    # Move group node
    
    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            ros2_control_node,
        ]
        + spawn_controllers
        + [generate_rsp_launch(moveit_config)]
        + [generate_move_group_launch(moveit_config)]
    )
