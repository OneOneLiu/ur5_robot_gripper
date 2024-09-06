#include "ur5_robot_gripper/robot_control.hpp"


int main(int argc, char * argv[]) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);

    // Create a node with the name "robot_control"
    auto node = std::make_shared<RobotMover>(rclcpp::NodeOptions());

    // Spin the node
    rclcpp::spin(node);

    // Shutdown the ROS node
    rclcpp::shutdown();
  return 0;
}
