#include "ur5_robot_gripper/robot_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

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
  std::cout << "4: Move in Cartesian path to a specified pose (position and orientation)\n"; // 新增选项
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
  } else if (choice == 4) {
    // 获取用户输入的目标位置和姿态
    double px, py, pz, qx, qy, qz, qw;
    std::cout << "Enter position (x, y, z): ";
    std::cin >> px >> py >> pz;

    // 使用笛卡尔路径移动到指定位置和姿态
    robot_mover.moveToCartesianPosition(px, py, pz);
  } else {
    std::cout << "Invalid choice. Exiting.\n";
  }

  // Shutdown ROS
  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}
