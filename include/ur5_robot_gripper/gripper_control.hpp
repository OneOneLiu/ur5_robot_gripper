#ifndef GRIPPER_CONTROL_HPP
#define GRIPPER_CONTROL_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ur5_robot_gripper/action/move_gripper_action.hpp"  // Action definition for moving the gripper

// Type definitions for MoveGripperAction
using MoveGripperAction = ur5_robot_gripper::action::MoveGripperAction;
using GoalHandleMoveGripperAction = rclcpp_action::ServerGoalHandle<MoveGripperAction>;

class GripperControl : public rclcpp::Node
{
public:
    // Constructor
    explicit GripperControl(const rclcpp::NodeOptions &options);

private:
    // Action server for moving the gripper
    rclcpp_action::Server<MoveGripperAction>::SharedPtr action_server_;

    // Helper method to execute a motion plan
    void executeGripperPlan();

    // Action goal handling functions
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveGripperAction::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle);
    void executeGoal(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle);

    // MoveGroup interface for controlling the gripper
    moveit::planning_interface::MoveGroupInterface gripper_move_group_interface_;
};

#endif // GRIPPER_CONTROL_HPP
