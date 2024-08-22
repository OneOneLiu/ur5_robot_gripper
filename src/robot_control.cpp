#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class RobotMover
{
public:
  RobotMover(const rclcpp::Node::SharedPtr& node)
  : move_group_interface_(node, "manipulator")
  {
    // 初始化 MoveGroupInterface
  }

  // 方法1：移动到指定位置和姿态
  void moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw)
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

  // 方法2：保持当前姿态，移动到指定的位置
  void moveToPosition(double px, double py, double pz)
  {
    auto current_pose = move_group_interface_.getCurrentPose().pose;
    current_pose.position.x = px;
    current_pose.position.y = py;
    current_pose.position.z = pz;

    move_group_interface_.setPoseTarget(current_pose);
    executePlan();
  }
  // 方法3：打印当前的姿态
  void printCurrentPose()
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

private:
  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  void executePlan()
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
};

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create and start the multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  // 创建 RobotMover 实例
  RobotMover robot_mover(node);

  // 选择调用的方法
  std::cout << "Choose the method to call:\n";
  std::cout << "1: Move to a specified pose (position and orientation)\n";
  std::cout << "2: Move to a position while maintaining the current orientation\n";
  std::cout << "3: Print the current pose\n";
  int choice;
  std::cin >> choice;

  if (choice == 1) {
    // 获取用户输入的目标位置和姿态
    double px, py, pz, qx, qy, qz, qw;
    std::cout << "Enter position (x, y, z): ";
    std::cin >> px >> py >> pz;
    std::cout << "Enter orientation (qx, qy, qz, qw): ";
    std::cin >> qx >> qy >> qz >> qw;

    // 使用方法1移动到指定位置和姿态
    robot_mover.moveToPose(px, py, pz, qx, qy, qz, qw);
  } else if (choice == 2) {
    // 获取用户输入的目标位置
    double px, py, pz;
    std::cout << "Enter position (x, y, z): ";
    std::cin >> px >> py >> pz;

    // 使用方法2保持当前姿态，移动到指定位置
    robot_mover.moveToPosition(px, py, pz);
  } else if (choice == 3) {
    // 打印当前的姿态
    robot_mover.printCurrentPose();
  } else {
    std::cout << "Invalid choice. Exiting.\n";
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
