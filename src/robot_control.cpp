#include "ur5_robot_gripper/robot_control.hpp"

// RobotMover 类的实现部分
RobotMover::RobotMover(const rclcpp::Node::SharedPtr& node)
: move_group_interface_(node, "manipulator")
{
  // 初始化 MoveGroupInterface
}

void RobotMover::moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = pz;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  move_group_interface_.setPoseTarget(target_pose);
  executePlan();
}

void RobotMover::moveToPosition(double px, double py, double pz)
{
  auto current_pose = move_group_interface_.getCurrentPose().pose;
  current_pose.position.x = px;
  current_pose.position.y = py;
  current_pose.position.z = pz;

  move_group_interface_.setPoseTarget(current_pose);
  executePlan();
}

void RobotMover::printCurrentPose()
{
  auto current_pose = move_group_interface_.getCurrentPose().pose;
  std::cout << "Current Pose:" << std::endl;
  std::cout << "Position: (" << current_pose.position.x << ", "
            << current_pose.position.y << ", "
            << current_pose.position.z << ")" << std::endl;
  std::cout << "Orientation: (" << current_pose.orientation.x << ", "
            << current_pose.orientation.y << ", "
            << current_pose.orientation.z << ", "
            << current_pose.orientation.w << ")" << std::endl;
}

void RobotMover::executePlan()
{
  auto const [success, plan] = [&]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    move_group_interface_.execute(plan);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("robot_control"), "Planning failed!");
  }
}

void RobotMover::moveInCartesianPathToPose(double px, double py, double pz, double qx, double qy, double qz, double qw)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group_interface_.getCurrentPose().pose;

  // 添加当前位置作为路径的起始点
  waypoints.push_back(start_pose);

  // 添加目标姿态作为路径的终止点
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = pz;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  waypoints.push_back(target_pose);

  moveInCartesianPath(waypoints);
}

void RobotMover::moveInCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::msg::RobotTrajectory trajectory;

  // 规划笛卡尔路径
  double fraction = move_group_interface_.computeCartesianPath(
    waypoints,    // 路径点列表
    0.01,         // eef_step，步长
    0.0,          // jump_threshold
    trajectory    // 返回的轨迹
  );

  if (fraction < 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_control"), "Cartesian path planning failed with fraction: %.2f", fraction);
    return;
  }

  // 设置规划的轨迹
  plan.trajectory_ = trajectory;

  // 执行规划
  move_group_interface_.execute(plan);
}

void RobotMover::moveToCartesianPosition(double px, double py, double pz)
{
  auto current_pose = move_group_interface_.getCurrentPose().pose;
  current_pose.position.x = px;
  current_pose.position.y = py;
  current_pose.position.z = pz;

  moveInCartesianPathToPose(current_pose.position.x, current_pose.position.y, current_pose.position.z,
                             current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
}