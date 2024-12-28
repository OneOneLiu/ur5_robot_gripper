#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include <memory>
#include <optional>  // 引入 std::optional
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "ur5_robot_gripper/srv/print_pose.hpp"  // Custom service for printing the pose
#include "ur5_robot_gripper/srv/move_to_position.hpp"  // Custom service for moving to a position
#include "ur5_robot_gripper/srv/set_constraints.hpp"  // Custom service for printing the pose
#include "ur5_robot_gripper/srv/move_to_pose.hpp"  // Custom service for moving to a position
#include "ur5_robot_gripper/action/move_to_position_action.hpp"  // MoveToPosition action definition
#include "ur5_robot_gripper/action/move_to_pose_action.hpp"  // MoveToPose action definition
#include "ur5_robot_gripper/action/move_to_joint_position.hpp"  // MoveToJointPosition action definition
#include <Eigen/Geometry> // For visualizing shapes
#include <iostream>
#include <vector>

// For debugging
#include <nlohmann/json.hpp> 
#include <fstream>

// Type definitions for MoveToPositionAction
using MoveToPositionAction = ur5_robot_gripper::action::MoveToPositionAction;
using GoalHandleMoveToPositionAction = rclcpp_action::ServerGoalHandle<MoveToPositionAction>;

// Type definitions for MoveToPoseAction
using MoveToPoseAction = ur5_robot_gripper::action::MoveToPoseAction;
using GoalHandleMoveToPoseAction = rclcpp_action::ServerGoalHandle<MoveToPoseAction>;

// Type definitions for MoveToJointPosition
using MoveToJointPosition = ur5_robot_gripper::action::MoveToJointPosition;
using GoalHandleMoveToJointPosition = rclcpp_action::ServerGoalHandle<MoveToJointPosition>;

class RobotMover : public rclcpp::Node
{
public:
    // Constructor
    explicit RobotMover(const rclcpp::NodeOptions &options);

    // Method to move to a specified pose
    bool moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw, double velocity_scaling);

    // Method to maintain current orientation and move to a specified position
    void moveToPosition(double px, double py, double pz, double velocity_scaling);

    // Method to move to a specified joint position
    void moveToJointPosition(const std::vector<double>& joint_angles, double velocity_scaling);

    // Method to print the current pose of the robot
    void printCurrentPose();
    
    // Method to setup constraints during planning
    void setConstraints(bool use_pos_cons,double box_dx, double box_dy, double box_dz, bool use_end_position, double box_px, double box_py, double box_pz, bool use_ori_cons, bool keep_end_orientation, double qx, double qy, double qz, double qw, double angle_tolerance);

    // For debugging
    void savePlanToJson(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_name);
    
    // Method to visualize a box in RViz
    void visualizeBox(const geometry_msgs::msg::Pose &box_pose, double box_dx, double box_dy, double box_dz);

private:
    // Helper method to execute a motion plan
    bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan);
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> genPlan(double velocity_scaling);
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

    // Service callback functions
    void getRobotStateRequest(const std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Request> /*request*/,
                          std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Response> response);
    void handleMovePositionRequest(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Response> response);
    void handleMovePoseRequest(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Response> response);
    bool handleSetConstraintsRequest(
                            const std::shared_ptr<ur5_robot_gripper::srv::SetConstraints::Request> request,
                            std::shared_ptr<ur5_robot_gripper::srv::SetConstraints::Response> response);
    // Service for setup motion planning constraints
    rclcpp::Service<ur5_robot_gripper::srv::SetConstraints>::SharedPtr set_constraint_service_;
    // Action for MoveToPosition
    rclcpp_action::Server<MoveToPositionAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPositionAction::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle);
    void executeGoal(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle);

    // Action for MoveToPose
    rclcpp_action::Server<MoveToPoseAction>::SharedPtr move_to_pose_action_server_;

    rclcpp_action::GoalResponse handlePoseGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPoseAction::Goal> goal);
    rclcpp_action::CancelResponse handlePoseCancel(const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle);
    void handlePoseAccepted(const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle);
    void executePoseGoal(const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle);

    // Action for MoveToJointPosition
    rclcpp_action::Server<MoveToJointPosition>::SharedPtr move_to_joint_position_action_server_;

    rclcpp_action::GoalResponse handleJointGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToJointPosition::Goal> goal);
    rclcpp_action::CancelResponse handleJointCancel(const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle);
    void handleJointAccepted(const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle);
    void executeJointGoal(const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle);

    // Member variables
    rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
    moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
    rclcpp::Service<ur5_robot_gripper::srv::PrintPose>::SharedPtr print_current_pose_service_;  // Service pointer for pose requests
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPosition>::SharedPtr move_to_position_service_;  //Service pointer for position move requests
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPose>::SharedPtr move_to_pose_service_;  // 
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
    std::thread executor_thread_;  // Thread to run the executor
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group_;
};

#endif // ROBOT_CONTROL_HPP
