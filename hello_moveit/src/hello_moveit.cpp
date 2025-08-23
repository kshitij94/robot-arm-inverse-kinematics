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

  /*
  while (rclcpp::ok())
  {
    // Set the current pose as the target
    auto const target_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "current pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
*/
  /*
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.position.x = -0.009225;
    msg.position.y = -0.000191;
    msg.position.z = 0.064217;
    msg.orientation.x = 0.464386;
    msg.orientation.y = -0.030455;
    msg.orientation.z = 0.883212;
    msg.orientation.w = 0.057916;
    return msg;
  }();
  */
  auto const target_pose = move_group_interface.getCurrentPose().pose;

  RCLCPP_INFO(logger, "current pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  move_group_interface.setNumPlanningAttempts(10); // Increase planning attempts
  move_group_interface.setPoseTarget(target_pose);
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
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}