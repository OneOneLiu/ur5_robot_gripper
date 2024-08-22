#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("robot_control");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Parse command-line arguments for the target pose
  if (argc < 7) {
    RCLCPP_ERROR(logger, "Usage: robot_moveit <px> <py> <pz> <qx> <qy> <qz> <qw>");
    return 1;
  }

  auto const target_pose = [&]{
    geometry_msgs::msg::Pose msg;
    msg.position.x = std::stof(argv[1]);
    msg.position.y = std::stof(argv[2]);
    msg.position.z = std::stof(argv[3]);
    msg.orientation.x = std::stof(argv[4]);
    msg.orientation.y = std::stof(argv[5]);
    msg.orientation.z = std::stof(argv[6]);
    msg.orientation.w = std::stof(argv[7]);
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}