#include "ur5_robot_gripper/gripper_control.hpp"

GripperControl::GripperControl(const rclcpp::NodeOptions &options)
  : Node("gripper_control", options),
    gripper_move_group_interface_(std::make_shared<rclcpp::Node>("gripper_moveit"), "gripper")
{
    // Create the action server
    this->action_server_ = rclcpp_action::create_server<MoveGripperAction>(
      this,
      "move_gripper_action",
      std::bind(&GripperControl::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperControl::handleCancel, this, std::placeholders::_1),
      std::bind(&GripperControl::handleAccepted, this, std::placeholders::_1)
    );
}

rclcpp_action::GoalResponse GripperControl::handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveGripperAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received action goal to move gripper to %.2f", goal->target_position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperControl::handleCancel(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for gripper movement");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperControl::handleAccepted(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle)
{
    std::thread([this, goal_handle]() {
        executeGoal(goal_handle);
    }).detach();
}

void GripperControl::executeGoal(const std::shared_ptr<GoalHandleMoveGripperAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing gripper action goal...");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveGripperAction::Feedback>();
    auto result = std::make_shared<MoveGripperAction::Result>();

    // Set the target position for the gripper (e.g., closing the gripper)
    std::map<std::string, double> target_joint_values;
    target_joint_values["robotiq_85_left_knuckle_joint"] = goal->target_position;

    gripper_move_group_interface_.setJointValueTarget(target_joint_values);

    // Plan the motion
    auto const [gripper_success, gripper_plan] = [&]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(gripper_move_group_interface_.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // If the plan is successful, execute it
    if (gripper_success) {
        RCLCPP_INFO(this->get_logger(), "Gripper action plan succeeded. Executing...");
        gripper_move_group_interface_.execute(gripper_plan);
        result->success = true;
        goal_handle->succeed(result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Gripper action plan failed!");
        result->success = false;
        goal_handle->abort(result);
    }
}

