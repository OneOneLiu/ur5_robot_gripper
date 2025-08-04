#include "ur5_robot_gripper/robot_control_real.hpp"

// RobotMover class implementation
RobotMover::RobotMover(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control_real", options), // Initialize the node with the name "robot_control_real"
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
    // 创建一个服务，用于打印当前姿态
    print_current_pose_service_ = this->create_service<ur5_robot_gripper::srv::PrintPose>(
      "print_current_pose", 
      std::bind(&RobotMover::getRobotStateRequest, this, std::placeholders::_1, std::placeholders::_2)
    );

    set_constraint_service_ = this->create_service<ur5_robot_gripper::srv::SetConstraints>(
    "set_constraints", std::bind(&RobotMover::handleSetConstraintsRequest, this, std::placeholders::_1, std::placeholders::_2));

    // 创建一个话题，用于发布当前姿态
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/current_robot_pose", 10);

    // 创建一个 100ms 周期的定时器，回调里读 move_group_interface_ 并发布
    pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
        // 从 MoveIt 读当前 pose（返回的是 PoseStamped）
        auto current = move_group_interface_.getCurrentPose();

        // 填一下 header
        current.header.stamp = this->now();
        current.header.frame_id = move_group_interface_.getPlanningFrame();

        // 发布
        pose_pub_->publish(current);
        RCLCPP_DEBUG(this->get_logger(),
                    "Published current_pose: [%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
                    current.pose.position.x,
                    current.pose.position.y,
                    current.pose.position.z,
                    current.pose.orientation.x,
                    current.pose.orientation.y,
                    current.pose.orientation.z,
                    current.pose.orientation.w);
        });
    
    // 创建用于控制机器人运动到指定位置的Action Server
    this->move_to_position_action_server_ = rclcpp_action::create_server<MoveToPositionAction>(
      this,
      "move_to_position_action", // 与服务区分开
      std::bind(&RobotMover::handleMovePositionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handleMovePositionCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handleMovePositionAccepted, this, std::placeholders::_1)
    );

    // 创建用于控制机器人运动到指定姿态的Action Server
    this->move_to_pose_action_server_ = rclcpp_action::create_server<MoveToPoseAction>(
      this,
      "move_to_pose_action",
      std::bind(&RobotMover::handlePoseGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handlePoseCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handlePoseAccepted, this, std::placeholders::_1)
    );
    // 创建用于控制机器人运动到指定关节位置的Action Server
    this->move_to_joint_position_action_server_ = rclcpp_action::create_server<MoveToJointPositionAction>(
      this,
      "move_to_joint_position_action",
      std::bind(&RobotMover::handleJointGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotMover::handleJointCancel, this, std::placeholders::_1),
      std::bind(&RobotMover::handleJointAccepted, this, std::placeholders::_1)
  );
    // 将节点添加到执行器并启动执行器线程
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
      executor_->spin(); // Run the executor to process callbacks
    });
}

// 函数用于可视化约束框
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

// 函数用于打印当前末端执行器姿态和关节角度
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

// 函数用于控制机器人运动到指定姿态
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
  
  // 设置速度和加速度缩放因子
  move_group_interface_.setMaxVelocityScalingFactor(velocity_scaling);
  move_group_interface_.setMaxAccelerationScalingFactor(velocity_scaling);
  
  // 使用move()方法自动完成规划和执行
  auto result = move_group_interface_.move();
  
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Move failed with error code: %d", result.val);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Move completed successfully");
  return true;
}

// 函数用于控制机器人保持当前姿态，运动到指定位置
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
  
  // 设置速度和加速度缩放因子
  move_group_interface_.setMaxVelocityScalingFactor(velocity_scaling);
  move_group_interface_.setMaxAccelerationScalingFactor(velocity_scaling);
  
  // 使用move()方法自动完成规划和执行
  auto result = move_group_interface_.move();
  
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Move failed with error code: %d", result.val);
  } else {
    RCLCPP_INFO(this->get_logger(), "Move completed successfully");
  }
}

void RobotMover::getRobotStateRequest(
    const std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Request> request,
    std::shared_ptr<ur5_robot_gripper::srv::PrintPose::Response> response) {
    
    // 获取当前姿态和关节角
    auto current_pose = move_group_interface_.getCurrentPose().pose;
    auto current_joint_values = move_group_interface_.getCurrentJointValues();

    // 填充响应
    response->pose = current_pose;
    response->joint_angles = current_joint_values;
    response->success = true;

    // 设置当前时间戳作为实际时间戳
    response->actual_timestamp = this->get_clock()->now();
}

// Action goal处理函数
rclcpp_action::GoalResponse RobotMover::handleMovePositionGoal([[maybe_unused]] const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPositionAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received action goal to move to position (x=%.2f, y=%.2f, z=%.2f)", 
                goal->px, goal->py, goal->pz);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Action取消处理函数
rclcpp_action::CancelResponse RobotMover::handleMovePositionCancel([[maybe_unused]] const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Action执行函数
void RobotMover::handleMovePositionAccepted(const std::shared_ptr<GoalHandleMoveToPositionAction> goal_handle)
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

    // bool reachable = isPoseReachableWithCollisionCheck(goal->px, goal->py, goal->pz, goal->qx, goal->qy, goal->qz, goal->qw);
    
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
bool RobotMover::moveToJointPosition(const std::vector<double>& joint_angles, double velocity_scaling) {
    move_group_interface_.setJointValueTarget(joint_angles); // Set target joint positions
    
    // 设置速度和加速度缩放因子
    move_group_interface_.setMaxVelocityScalingFactor(velocity_scaling);
    move_group_interface_.setMaxAccelerationScalingFactor(velocity_scaling);
    
    // 使用move()方法自动完成规划和执行
    auto result = move_group_interface_.move();
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Move failed with error code: %d", result.val);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Move completed successfully");
    return true;
}

// Goal handling function for MoveToJointPosition
// test: ros2 action send_goal /move_to_joint_position_action ur5_robot_gripper/action/MoveToJointPosition "{joint_positions: [0, -1.57, 1.57, -1.57, -1.57, 0], velocity_scaling: 0.5}"

rclcpp_action::GoalResponse RobotMover::handleJointGoal([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const MoveToJointPositionAction::Goal> goal) {
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
    auto feedback = std::make_shared<MoveToJointPositionAction::Feedback>();
    auto result = std::make_shared<MoveToJointPositionAction::Result>();

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
    orientation_constraint.header.frame_id = move_group_interface_.getPoseReferenceFrame(); // Use the base link as the reference frame
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
    // orientation_constraint.absolute_z_axis_tolerance = angle_tolerance;
    orientation_constraint.absolute_z_axis_tolerance = std::numeric_limits<double>::infinity(); // Do not constrain world Z axis rotation: 
    // https://github.com/moveit/moveit2/issues/2614
    // https://github.com/moveit/moveit2/pull/2775
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
    // I think 20 s should be enough for most cases, if the planner cannot sovle the problem in 20 s, it may not be able to solve it in a longer time.
    move_group_interface_.setPlanningTime(3.0);

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
