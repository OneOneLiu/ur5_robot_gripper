#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gripper_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gripper_control");

  using moveit::planning_interface::MoveGroupInterface;

  // 创建控制夹爪的MoveGroup接口
  auto gripper_move_group_interface = MoveGroupInterface(node, "gripper");
  
  // 设置目标关节位置（例如，关闭夹爪的目标位置）
  std::map<std::string, double> target_joint_values;
  target_joint_values["robotiq_85_left_knuckle_joint"] = 0.8;  // 根据需要设置目标角度
  
  // 将目标关节位置设置为MoveGroup的目标
  gripper_move_group_interface.setJointValueTarget(target_joint_values);
  
  // 规划到目标位置的路径
  auto const [gripper_success, gripper_plan] = [&gripper_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(gripper_move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 执行计划
  if(gripper_success) {
    gripper_move_group_interface.execute(gripper_plan);
  } else {
    RCLCPP_ERROR(logger, "Gripper planning failed!");
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}