#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class RobotMover
{
public:
    // Constructor
    explicit RobotMover(const rclcpp::Node::SharedPtr& node);

    // Method to move to a specified pose
    void moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw);

    // Method to maintain current orientation and move to a specified position
    void moveToPosition(double px, double py, double pz);

    // Method to print the current pose of the robot
    void printCurrentPose();

    // New method to move in a Cartesian path to a specified pose
    void moveInCartesianPathToPose(double px, double py, double pz, double qx, double qy, double qz, double qw);
    // Method to move in a Cartesian path
    void moveInCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    // Method to move to a Cartesian position
    void moveToCartesianPosition(double px, double py, double pz);

private:
    // Helper method to execute a motion plan
    void executePlan();

    // Member variables
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
};

#endif // ROBOT_CONTROL_HPP
