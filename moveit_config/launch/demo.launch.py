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
# from moveit_configs_utils.launches import generate_rsp_launch,generate_move_group_launch # Remove this import


def generate_launch_description():
    # Declare arguments for use_sim_time
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("roboturdf", package_name="moveit_config")
        .robot_description(
            file_path="config/roboturdf.urdf.xacro",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits(file_path="config/joint_limits.yaml") # Add this line
        .to_moveit_configs(
        )
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

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[moveit_config.robot_description, {
                "publish_frequency": 15.0,
            },
        ],
    )

    # Move Group Node (explicitly defined)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.pilz_cartesian_limits,
            moveit_config.joint_limits, # Add this line
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"monitor_dynamics": False},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        declared_arguments +
        [
            rviz_node,
            static_tf,
            ros2_control_node,
            robot_state_publisher, # Add robot_state_publisher
            move_group_node,       # Add the new move_group_node
        ]
        + spawn_controllers
        # + [generate_rsp_launch(moveit_config)] # Remove this
        # + [generate_move_group_launch(moveit_config)] # Remove this
    )