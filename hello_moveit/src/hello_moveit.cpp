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
  if (last_joint_state_msg) {
    RCLCPP_INFO(logger, "Last received joint state message timestamp: %f seconds", rclcpp::Time(last_joint_state_msg->header.stamp).seconds());
  } else {
    RCLCPP_INFO(logger, "No joint state message received yet.");
  }
  rclcpp::sleep_for(std::chrono::seconds(10));

  // Create a publisher to visualize the target pose
  auto const pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

  while (rclcpp::ok())
  {
    // Set the current pose as the target
    auto const target_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(logger, "current pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    auto const target_pose_stamped = [&]
    {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = move_group_interface.getPlanningFrame();
      msg.header.stamp = node->get_clock()->now();
      msg.pose = target_pose;
      return msg;
    }();
    pose_publisher->publish(target_pose_stamped);

    rclcpp::sleep_for(std::chrono::seconds(10));
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}