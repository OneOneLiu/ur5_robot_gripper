#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ur5_robot_gripper/srv/move_to_position.hpp"
#include "ur5_robot_gripper/srv/print_pose.hpp"
#include "ur5_robot_gripper/robot_control.hpp"

int main(int argc, char** argv)
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);

    // 创建一个RobotMover实例
    auto robot_mover = std::make_shared<RobotMover>(rclcpp::NodeOptions());

    // 打印菜单
    int option;
    double px, py, pz, qx, qy, qz, qw;

    do {
        std::cout << "============================\n";
        std::cout << "Choose an option:\n";
        std::cout << "1. Move to specified pose\n";
        std::cout << "2. Move to specified position\n";
        std::cout << "3. Print current pose\n";
        std::cout << "0. Exit\n";
        std::cout << "============================\n";
        std::cin >> option;

        switch (option) {
            case 1: {
                std::cout << "Enter target pose (position x y z, orientation qx qy qz qw):\n";
                std::cin >> px >> py >> pz >> qx >> qy >> qz >> qw;
                robot_mover->moveToPose(px, py, pz, qx, qy, qz, qw);
                break;
            }
            case 2: {
                std::cout << "Enter target position (x y z):\n";
                std::cin >> px >> py >> pz;
                robot_mover->moveToPosition(px, py, pz);
                break;
            }
            case 3: {
                robot_mover->printCurrentPose();
                break;
            }
            case 0: {
                std::cout << "Exiting...\n";
                break;
            }
            default: {
                std::cout << "Invalid option. Please choose again.\n";
                break;
            }
        }
    } while (option != 0);

    // 关闭ROS 2节点
    rclcpp::shutdown();
    return 0;
}
