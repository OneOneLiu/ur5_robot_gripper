#include "ur5_robot_gripper/robot_control.hpp"

// RobotMover class implementation
RobotMover::RobotMover(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control", options), // Initialize the node with the name "robot_control"
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "manipulator"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()), // Create a single-threaded executor
    visual_tools_(                         // Initialize MoveItVisualTools for visualization
          node_,              // Node shared pointer
          "base_link",                     // Base frame
          "/move_group_tutorial",    // Marker topic NOTE: this topic is published by the visual tools and
          move_group_interface_.getRobotModel() // Robot model
      )
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
    
    set_constraint_service_ = this->create_service<ur5_robot_gripper::srv::SetConstraints>(
    "set_constraints", std::bind(&RobotMover::handleSetConstraintsRequest, this, std::placeholders::_1, std::placeholders::_2));


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

// Function to visualize the box constraint
void RobotMover::visualizeBox(const geometry_msgs::msg::Pose &box_pose, double box_dx, double box_dy, double box_dz)
{
    // Define the box dimensions
    Eigen::Vector3d box_size(box_dx, box_dy, box_dz);

    // Add custom transparency by modifying the alpha channel
    std_msgs::msg::ColorRGBA color_with_alpha;
    color_with_alpha.r = 0.5; // Grey (R=G=B)
    color_with_alpha.g = 0.5;
    color_with_alpha.b = 0.5;
    color_with_alpha.a = 0.5; // Semi-transparent (alpha = 0.5)
    // Publish the box marker
    visual_tools_.publishCuboid(box_pose, box_size.x(), box_size.y(), box_size.z(), color_with_alpha);
    visual_tools_.trigger(); // Send markers to RViz
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
  joint_model_group_ = move_group_interface_.getCurrentState()->getJointModelGroup("manipulator");
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group_);
  visual_tools_.trigger();
  RCLCPP_INFO(rclcpp::get_logger("robot_control"), "Visualized the plan in Rviz");
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
    // RCLCPP_INFO(node_->get_logger(), "Service Callback: Current Pose and Joint Angles:");
    // printCurrentPose(); // Print pose and joint angles

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
        bool reachable = isPoseReachableWithCollisionCheck(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw);

        if (!reachable)
        {
            RCLCPP_ERROR(this->get_logger(), "The target pose is not reachable or is in collision.");
            response->success = false;
            response->message = "Target pose is not reachable or in collision.";
            return;
        }

        bool success = moveToPose(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw, request->velocity_scaling);
        
        // 保存默认规划器
        std::string default_planner = move_group_interface_.getPlannerId();

        // 定义规划器列表
        std::vector<std::string> planners = {
            "SBL", "LBKPIECE", "BKPIECE", "KPIECE", "RRT",
            "RRTConnect", "RRTstar", "TRRT", "PRM", "PRMstar"
        };
        if(!success){
            RCLCPP_WARN(this->get_logger(), "Motion plan failed using default planner, trying other planners.");

            // 遍历规划器列表
            for (const auto &planner : planners)
            {
                // 设置当前规划器
                move_group_interface_.setPlannerId(planner);
                RCLCPP_INFO(this->get_logger(), "Trying planner: %s", planner.c_str());

                // 尝试两次
                for (int attempt = 1; attempt <= 2; ++attempt)
                {
                    RCLCPP_INFO(this->get_logger(), "Attempt %d with planner %s", attempt, planner.c_str());
                    success = moveToPose(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw, request->velocity_scaling);
                    if (success)
                    {
                        RCLCPP_INFO(this->get_logger(), "Motion plan succeeded using planner %s on attempt %d.", planner.c_str(), attempt);
                        response->success = true;
                        response->message = "Motion plan generated successfully.";
                        response->trajectory = current_plan_.trajectory_.joint_trajectory;
                        // 恢复默认规划器
                        move_group_interface_.setPlannerId(default_planner);
                        return;
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Motion plan failed using planner %s on attempt %d.", planner.c_str(), attempt);
                    }
                }
            }
            // 恢复默认规划器
            move_group_interface_.setPlannerId(default_planner);
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

    bool reachable = isPoseReachableWithCollisionCheck(goal->px, goal->py, goal->pz, goal->qx, goal->qy, goal->qz, goal->qw);
    
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

// Constrained planning

void RobotMover::setConstraints(bool use_pos_cons,double box_dx, double box_dy, double box_dz, bool use_end_position, double box_px, double box_py, double box_pz, bool use_ori_cons, bool keep_end_orientation, double qx, double qy, double qz, double qw, double angle_tolerance) {
    // Get the current pose
    auto current_pose = move_group_interface_.getCurrentPose().pose;

    // Define the box constraint
    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = move_group_interface_.getPoseReferenceFrame();
    box_constraint.link_name = move_group_interface_.getEndEffectorLink();

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {box_dx, box_dy, box_dz}; // Box dimensions: width, height, depth
    box_constraint.constraint_region.primitives.emplace_back(box);

    // Set the pose of the box constraint, use current end pose
    geometry_msgs::msg::Pose box_pose;
    // Use the current end effector pose, if not, use the world pose
    if (use_end_position) {
            box_pose.position.x = current_pose.position.x;
            box_pose.position.y = current_pose.position.y;
            box_pose.position.z = current_pose.position.z;
            box_pose.orientation.w = current_pose.orientation.w;
            box_pose.orientation.x = current_pose.orientation.x;
            box_pose.orientation.y = current_pose.orientation.y;
            box_pose.orientation.z = current_pose.orientation.z;
    }
    else {
        box_pose.position.x = box_px;
        box_pose.position.y = box_py;
        box_pose.position.z = box_pz;
        box_pose.orientation.w = 1.0;
    }

    box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    box_constraint.weight = 1.0;

    // Create orientation constraint
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group_interface_.getPoseReferenceFrame();
    orientation_constraint.link_name = move_group_interface_.getEndEffectorLink();

    if (keep_end_orientation) {
        orientation_constraint.orientation = current_pose.orientation;
    }
    else {
        orientation_constraint.orientation.w = qw;
        orientation_constraint.orientation.x = qx;
        orientation_constraint.orientation.y = qy;
        orientation_constraint.orientation.z = qz;
    }

    orientation_constraint.absolute_x_axis_tolerance = angle_tolerance;
    orientation_constraint.absolute_y_axis_tolerance = angle_tolerance;
    orientation_constraint.absolute_z_axis_tolerance = angle_tolerance;
    orientation_constraint.weight = 1.0;

    // Create the constraints message
    moveit_msgs::msg::Constraints constraints;
    if (use_pos_cons) {
        constraints.position_constraints.emplace_back(box_constraint);
    }
    if (use_ori_cons) {
        constraints.orientation_constraints.emplace_back(orientation_constraint);
    }

    // Apply the constraints to the MoveGroupInterface
    move_group_interface_.setPathConstraints(constraints);
    // It’s helpful to increase the default planning time, as planning with constraints can be slower.
    move_group_interface_.setPlanningTime(60.0);

    // Visualize the box constraint in RViz
    visualizeBox(box_pose, box_dx, box_dy, box_dz);
}

bool RobotMover::handleSetConstraintsRequest(
    const std::shared_ptr<ur5_robot_gripper::srv::SetConstraints::Request> request,
    std::shared_ptr<ur5_robot_gripper::srv::SetConstraints::Response> response) 
{
    if (!request->use_pos_cons && !request->use_ori_cons)
    {
        move_group_interface_.clearPathConstraints();
        RCLCPP_INFO(this->get_logger(), "Cleared Constraints");
        visual_tools_.deleteAllMarkers();
        response->message = "Constraints cleared successfully.";
        response->success = true;
        return true;
    }
    
    setConstraints(request->use_pos_cons, request->box_dx, request->box_dy, request->box_dz, request->use_end_position, request->box_px, request->box_py, request->box_pz, request->use_ori_cons, request->keep_end_orientation, request->qx, request->qy, request->qz, request->qw, request->angle_tolerance);
    // Set the response message
    response->success = true;
    response->message = "Constraints set successfully.";
    RCLCPP_INFO(this->get_logger(), "Constraints set: Box [%f, %f, %f] at current robot end position",
                request->box_dx, request->box_dy, request->box_dz);
    
    return true;
}

bool RobotMover::isPoseReachableWithCollisionCheck(double px, double py, double pz, double qx, double qy, double qz, double qw)
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = px;
    target_pose.position.y = py;
    target_pose.position.z = pz;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    // 使用当前节点的共享指针初始化 RobotModelLoader
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");

    // 加载机器人模型
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    if (!kinematic_model)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load robot model!");
        return false;
    }

    // 创建 RobotState 和 PlanningScene
    moveit::core::RobotState kinematic_state(kinematic_model);
    kinematic_state.setToDefaultValues();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // 获取关节组
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    if (!joint_model_group)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group for manipulator!");
        return false;
    }

    // 尝试为目标位姿计算 IK 解
    bool found_ik = kinematic_state.setFromIK(joint_model_group, target_pose);
    if (!found_ik)
    {
        RCLCPP_WARN(this->get_logger(), "The target pose is not reachable (IK solution not found).");
        return false;
    }

    // 检查碰撞
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkCollision(collision_request, collision_result, kinematic_state);

    std::string collision_object = "";
    // 判断是否发生碰撞
    if (collision_result.collision)
    {
        RCLCPP_WARN(this->get_logger(), "Collision detected!");

        // 获取碰撞对象
        for (const auto& contact : collision_result.contacts)
        {
            collision_object = contact.first.first; // 碰撞的第一个对象
            RCLCPP_WARN(this->get_logger(), "Collision with: %s", collision_object.c_str());
        }
    }

    bool in_collision = planning_scene.isStateColliding(kinematic_state, "manipulator", true);
    if (in_collision)
    {
        RCLCPP_WARN(this->get_logger(), "The target pose is reachable but in collision.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "The target pose is reachable and collision-free.");
    return true;
}