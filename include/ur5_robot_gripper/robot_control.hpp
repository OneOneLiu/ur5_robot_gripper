#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "ur5_robot_gripper/srv/print_pose.hpp"  // Header file for the custom service
#include "ur5_robot_gripper/srv/move_to_position.hpp"
#include <iostream>

class RobotMover : public rclcpp::Node
{
public:
    // Constructor
    explicit RobotMover(const rclcpp::NodeOptions &options);

    // // Method to move to a specified pose
    void moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw);

    // Method to maintain current orientation and move to a specified position
    void moveToPosition(double px, double py, double pz);

    // Method to print the current pose of the robot
    void printCurrentPose();

private:
    // Helper method to execute a motion plan
    void executePlan();
    void handlePrintRequest(const std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Request> /*request*/,
                          std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Response> response);
    void handleMovePositionRequest(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Response> response);

    // Member variables
    rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
    moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
    rclcpp::Service<ur5_robot_gripper::srv::PrintPose>::SharedPtr print_current_pose_service_;  // Service pointer for pose requests
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPosition>::SharedPtr move_to_position_service_;  // Service pointer for position move requests
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
    std::thread executor_thread_;  // Thread to run the executor
};

#endif // ROBOT_CONTROL_HPP
