#include "ur5_robot_gripper/gripper_control.hpp"

int main(int argc, char *argv[]) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);

    // Create an instance of the GripperControl class
    auto node = std::make_shared<GripperControl>(rclcpp::NodeOptions());

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown the ROS node
    rclcpp::shutdown();
    return 0;
}
