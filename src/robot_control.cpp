#include "ur5_robot_gripper/robot_control.hpp"

// RobotMover class implementation
RobotMover::RobotMover(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control", options), // Initialize the node with the name "robot_control"
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "manipulator"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
{
    // Create the service for printing the current pose
    print_current_pose_service_ = this->create_service<ur5_robot_gripper::srv::PrintPose>(
      "print_current_pose", 
      std::bind(&RobotMover::getRobotStateRequest, this, std::placeholders::_1, std::placeholders::_2)
    );
    move_to_position_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPosition>(
            "move_to_position", std::bind(&RobotMover::handleMovePositionRequest, this, std::placeholders::_1, std::placeholders::_2));
    move_to_pose_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPose>(
            "move_to_pose", std::bind(&RobotMover::handleMovePoseRequest, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建 Action Server
    this->action_server_ = rclcpp_action::create_server<MoveToPositionAction>(
      this,
      "move_to_position_action", // 与服务区分开
      std::bind(&RobotMover::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handleCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handleAccepted, this, std::placeholders::_1)
    );

    // Create the action server for MoveToPoseAction
    this->move_to_pose_action_server_ = rclcpp_action::create_server<MoveToPoseAction>(
      this,
      "move_to_pose_action",
      std::bind(&RobotMover::handlePoseGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handlePoseCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handlePoseAccepted, this, std::placeholders::_1)
    );
    // Create the action server for MoveToJointPosition
    this->move_to_joint_position_action_server_ = rclcpp_action::create_server<MoveToJointPosition>(
      this,
      "move_to_joint_position_action",
      std::bind(&RobotMover::handleJointGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handleJointCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handleJointAccepted, this, std::placeholders::_1)
  );
    // Add the node to the executor and start the executor thread
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
      executor_->spin(); // Run the executor to process callbacks
    });
}

// For debugging
void RobotMover::savePlanToJson(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &file_name)
{
    nlohmann::json json_plan;

    for (const auto &point : plan.trajectory_.joint_trajectory.points)
    {
        nlohmann::json json_point;
        json_point["time_from_start"] = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        json_point["positions"] = point.positions;
        json_point["velocities"] = point.velocities;
        json_point["accelerations"] = point.accelerations;
        json_plan.push_back(json_point);
    }

    std::ofstream file(file_name);
    if (!file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_control"), "Failed to open file: %s", file_name.c_str());
        return;
    }

    file << json_plan.dump(4); // Save JSON with indentation
    file.close();

    RCLCPP_INFO(rclcpp::get_logger("robot_control"), "Motion plan saved to %s", file_name.c_str());
}

// Function to print the current end-effector pose and joint angles
void RobotMover::printCurrentPose() {
    auto current_pose = move_group_interface_.getCurrentPose().pose; // Get the current pose
    auto current_joint_values = move_group_interface_.getCurrentJointValues(); // Get the current joint angles

    // Print the pose
    std::cout << "Current Pose:" << std::endl;
    std::cout << "Position: (" << current_pose.position.x << ", "
              << current_pose.position.y << ", "
              << current_pose.position.z << ")" << std::endl;
    std::cout << "Orientation: (" << current_pose.orientation.x << ", "
              << current_pose.orientation.y << ", "
              << current_pose.orientation.z << ", "
              << current_pose.orientation.w << ")" << std::endl;

    // Print the joint angles
    std::cout << "Current Joint Angles:" << std::endl;
    for (size_t i = 0; i < current_joint_values.size(); ++i) {
        std::cout << "Joint " << i + 1 << ": " << current_joint_values[i] << std::endl;
    }
}

bool RobotMover::moveToPose(double px, double py, double pz, double qx, double qy, double qz, double qw, double velocity_scaling)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = pz;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  RCLCPP_INFO(this->get_logger(), "Moving to pose (x=%.8f, y=%.8f, z=%.8f, qx=%.8f, qy=%.8f, qz=%.8f, qw=%.8f)", px, py, pz, qx, qy, qz, qw);

  move_group_interface_.setPoseTarget(target_pose);
  auto plan_opt = genPlan(velocity_scaling);
  if (!plan_opt) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate motion plan.");
        return false;
    }
    // 解包 std::optional
    current_plan_ = *plan_opt;

    return true;
}

void RobotMover::moveToPosition(double px, double py, double pz, double velocity_scaling = 0.01)
{
  auto current_pose = move_group_interface_.getCurrentPose().pose;
  // 打印当前姿态的位置和方向
  RCLCPP_WARN(rclcpp::get_logger("robot_control"), 
                "Current pose before move - Position: x=%.3f, y=%.3f, z=%.3f; Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                current_pose.position.x, current_pose.position.y, current_pose.position.z,
                current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  current_pose.position.x = px;
  current_pose.position.y = py;
  current_pose.position.z = pz;

  RCLCPP_WARN(rclcpp::get_logger("robot_control"), "moveToPosition function called");
  RCLCPP_WARN(rclcpp::get_logger("robot_control"), 
                "Target pose - Position: x=%.3f, y=%.3f, z=%.3f; Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                px, py, pz,
                current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  move_group_interface_.setPoseTarget(current_pose);
  auto plan_opt = genPlan(velocity_scaling);  // 调用 genPlan = genPlan(velocity_scaling);
  if (!plan_opt) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate motion plan.");
    }
  // 解包 std::optional
  current_plan_ = *plan_opt;
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> 
RobotMover::genPlan(double velocity_scaling)
{
  move_group_interface_.setGoalOrientationTolerance(0.0001); // Radians, adjust as needed
  move_group_interface_.setGoalPositionTolerance(0.0001); // Meters, adjust as needed

  // Set velocity and acceleration scaling factors
  move_group_interface_.setMaxVelocityScalingFactor(velocity_scaling);
  move_group_interface_.setMaxAccelerationScalingFactor(velocity_scaling);

  // Create a plan object
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Generate the motion plan
  bool success = static_cast<bool>(move_group_interface_.plan(plan));

  // Save the plan to JSON for debugging
  savePlanToJson(plan, "motion_plan.json");

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_control"), "Planning failed!");
    return std::nullopt;  // 规划失败，返回空值
  }

  RCLCPP_INFO(rclcpp::get_logger("robot_control"), "Planning succeeded!");
  return plan;  // 返回生成的 plan
}

bool RobotMover::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
  // Execute the motion plan
  auto execute_status = move_group_interface_.execute(plan);

  if (execute_status != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_control"), "Execution failed!");
    return false;  // 执行失败
  }

  RCLCPP_INFO(rclcpp::get_logger("robot_control"), "Execution succeeded!");
  return true;  // 执行成功
}


// Service callback function to handle pose and joint angle printing requests
void RobotMover::getRobotStateRequest(const std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Request> /*request*/,
                                    std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Response> response) {
    // Get the current pose and joint angles
    auto current_pose = move_group_interface_.getCurrentPose().pose; 
    auto current_joint_values = move_group_interface_.getCurrentJointValues(); 

    // Print both pose and joint angles
    RCLCPP_INFO(node_->get_logger(), "Service Callback: Current Pose and Joint Angles:");
    printCurrentPose(); // Print pose and joint angles

    // Set the pose in the response
    response->pose.position.x = current_pose.position.x;
    response->pose.position.y = current_pose.position.y;
    response->pose.position.z = current_pose.position.z;
    response->pose.orientation.x = current_pose.orientation.x;
    response->pose.orientation.y = current_pose.orientation.y;
    response->pose.orientation.z = current_pose.orientation.z;
    response->pose.orientation.w = current_pose.orientation.w;

    // Set the joint angles in the response
    response->joint_angles = current_joint_values;

    // Set the response to indicate success
    response->success = true;
}

void RobotMover::handleMovePositionRequest(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Response> response)
    {
        // 延迟确保状态信息已经更新
        RCLCPP_INFO(this->get_logger(), "Get Pose in call.");
        printCurrentPose();  // 获取当前姿态
        moveToPosition(request->px, request->py, request->pz);
        response->success = true;
    }

void RobotMover::handleMovePoseRequest(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Response> response)
    {
        // 延迟确保状态信息已经更新
        RCLCPP_INFO(this->get_logger(), "Get Pose in call.");
        printCurrentPose();  // 获取当前姿态
        bool success = moveToPose(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw, request->velocity_scaling);

        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute motion plan.");
            response->success = false;
            response->message = "Failed to generate motion plan.";
            return;
        }

        else {
            response->success = true;
            response->message = "Motion plan generated successfully.";
            response->trajectory = current_plan_.trajectory_.joint_trajectory;
        }
    }

// Action goal处理函数
rclcpp_action::GoalResponse RobotMover::handleGoal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPositionAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received action goal to move to position (x=%.2f, y=%.2f, z=%.2f)", 
                goal->px, goal->py, goal->pz);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Action取消处理函数
rclcpp_action::CancelResponse RobotMover::handleCancel([[maybe_unused]] const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Action执行函数
void RobotMover::handleAccepted(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle)
{
    std::thread([this, goal_handle]() {
        executeGoal(goal_handle);
    }).detach();
}

// 执行运动并发布反馈
void RobotMover::executeGoal(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing action goal...");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPositionAction::Feedback>();
    auto result = std::make_shared<MoveToPositionAction::Result>();

    // 调用 moveToPosition 而不是直接调用 Action
    moveToPosition(goal->px, goal->py, goal->pz);

    // // 模拟运动执行反馈
    // for (int i = 0; i <= 100; ++i) {
    //     if (goal_handle->is_canceling()) {
    //         result->success = false;
    //         goal_handle->canceled(result);
    //         RCLCPP_INFO(this->get_logger(), "Action goal canceled");
    //         return;
    //     }

    //     feedback->percentage_complete = i;
    //     goal_handle->publish_feedback(feedback);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action goal completed successfully");
}

// Action goal handling function for MoveToPoseAction
rclcpp_action::GoalResponse RobotMover::handlePoseGoal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPoseAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received action goal to move to pose (x=%.8f, y=%.8f, z=%.8f, qx=%.8f, qy=%.8f, qz=%.8f, qw=%.8f)", 
                goal->px, goal->py, goal->pz, goal->qx, goal->qy, goal->qz, goal->qw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Action cancel handling function for MoveToPoseAction
rclcpp_action::CancelResponse RobotMover::handlePoseCancel([[maybe_unused]] const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for move to pose");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Action accepted handling function for MoveToPoseAction
void RobotMover::handlePoseAccepted(const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle)
{
    std::thread([this, goal_handle]() {
        executePoseGoal(goal_handle);
    }).detach();
}

// Execute the goal and publish feedback
void RobotMover::executePoseGoal(const std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing action goal...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPoseAction::Feedback>();
    auto result = std::make_shared<MoveToPoseAction::Result>();

    // 调用 moveToPose
    // moveToPose 执行完毕后，检查规划和执行结果
    if (moveToPose(goal->px, goal->py, goal->pz, goal->qx, goal->qy, goal->qz, goal->qw, goal->velocity_scaling)) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action goal completed successfully");
    } else {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Failed to execute motion plan");
    }
}

// Add a method for move to joint position
// Function to move the robot to a specific joint position
void RobotMover::moveToJointPosition(const std::vector<double>& joint_angles, double velocity_scaling) {
    move_group_interface_.setJointValueTarget(joint_angles); // Set target joint positions
    auto plan_opt = genPlan(velocity_scaling); // Generate the motion plan
    if (!plan_opt) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate motion plan.");
    }
    // 解包 std::optional
    current_plan_ = *plan_opt;
}

// Goal handling function for MoveToJointPosition
// test: ros2 action send_goal /move_to_joint_position_action ur5_robot_gripper/action/MoveToJointPosition "{joint_positions: [0, -1.57, 1.57, -1.57, -1.57, 0], velocity_scaling: 0.5}"

rclcpp_action::GoalResponse RobotMover::handleJointGoal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const MoveToJointPosition::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received action goal to move to joint positions");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel handling function for MoveToJointPosition
rclcpp_action::CancelResponse RobotMover::handleJointCancel([[maybe_unused]] const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for move to joint positions");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Accepted goal function for MoveToJointPosition
void RobotMover::handleJointAccepted(const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle) {
    std::thread([this, goal_handle]() {
        executeJointGoal(goal_handle);
    }).detach();
}

// Execute the joint position action and publish feedback
void RobotMover::executeJointGoal(const std::shared_ptr<GoalHandleMoveToJointPosition> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing move to joint positions action goal...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToJointPosition::Feedback>();
    auto result = std::make_shared<MoveToJointPosition::Result>();

    // Move to the specified joint positions
    moveToJointPosition(goal->joint_positions, goal->velocity_scaling);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Move to joint positions action goal completed successfully");
}