#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

    // Wait for the CurrentStateMonitor to receive joint states
  RCLCPP_INFO(logger, "Waiting for 2 seconds for CurrentStateMonitor to get joint states...");
  rclcpp::sleep_for(std::chrono::seconds(10));

  // Set the current pose as the target
  auto const target_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Target pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  // Create a publisher to visualize the target pose
  auto const pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  auto const target_pose_stamped = [&]
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = move_group_interface.getPlanningFrame();
    msg.header.stamp = node->get_clock()->now();
    msg.pose = target_pose;
    return msg;
  }();
  pose_publisher->publish(target_pose_stamped);

  // Set the Pose target
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}