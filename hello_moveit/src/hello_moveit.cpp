#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

// Global variable to store the last received joint state message
sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg;

void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  last_joint_state_msg = msg;
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;

  joint_state_subscription = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, &topic_callback);

  std::thread spin_thread([node]()
                          { rclcpp::spin(node); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Wait for the CurrentStateMonitor to receive joint states
  RCLCPP_INFO(logger, "Waiting for MoveGroupInterface to receive current state...");
  move_group_interface.setStartStateToCurrentState();

  // 1. hello_moveit will store the current pose in a variable. Lets call this poseA.
  geometry_msgs::msg::Pose poseA = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Stored initial pose (poseA): x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              poseA.position.x, poseA.position.y, poseA.position.z,
              poseA.orientation.x, poseA.orientation.y, poseA.orientation.z, poseA.orientation.w);

  // Define a hardcoded poseB from the user's RViz input
  geometry_msgs::msg::Pose zeroPose;
  zeroPose.position.x = -0.009716;
  zeroPose.position.y = -0.000445;
  zeroPose.position.z = 0.064217;
  zeroPose.orientation.x = -0.620928;
  zeroPose.orientation.y = 0.023638;
  zeroPose.orientation.z = 0.782944;
  zeroPose.orientation.w = 0.029811;

  // Plan and move to poseB
  RCLCPP_INFO(logger, "Attempting to plan and move to zero position...");
  RCLCPP_INFO(logger, "Stored initial pose (zeroPose): x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              zeroPose.position.x, zeroPose.position.y, zeroPose.position.z,
              zeroPose.orientation.x, zeroPose.orientation.y, zeroPose.orientation.z, zeroPose.orientation.w);

  move_group_interface.setPoseTarget(zeroPose);
  moveit::planning_interface::MoveGroupInterface::Plan zeroPlan;
  bool success = (move_group_interface.plan(zeroPlan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan to move to poseB.");
    move_group_interface.execute(zeroPlan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed to move to poseB!");
  }

  // Wait for a moment
  rclcpp::sleep_for(std::chrono::seconds(5));

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}