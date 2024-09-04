#include "ur5_robot_gripper/robot_control.hpp"
#include "ur5_robot_gripper/srv/move_to_pose.hpp"
#include "ur5_robot_gripper/srv/move_to_position.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

class RobotServiceNode : public rclcpp::Node
{
public:
    explicit RobotServiceNode(std::shared_ptr<RobotMover> robot_mover)
    : Node("robot_service_node"), robot_mover_(robot_mover)
    {
        move_to_pose_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPose>(
            "move_to_pose", std::bind(&RobotServiceNode::moveToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

        move_to_position_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPosition>(
            "move_to_position", std::bind(&RobotServiceNode::moveToPositionCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "RobotServiceNode services are ready.");

        // 确保状态同步完成后再查询
        this->waitForRobotState();
        RCLCPP_INFO(this->get_logger(), "Get Pose in init.");
        robot_mover_->printCurrentPose();  // 获取当前姿态
    }

private:
    void waitForRobotState()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot state...");
        // 等待一定时间，确保 joint_states 话题开始发布
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    void moveToPoseCallback(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Request> request,
                            std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Response> response)
    {
        robot_mover_->moveToPose(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw);
        response->success = true;
    }

    void moveToPositionCallback(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Response> response)
    {
        // 延迟确保状态信息已经更新
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(this->get_logger(), "Get Pose in call.");
        robot_mover_->printCurrentPose();  // 获取当前姿态
        robot_mover_->moveToPosition(request->px, request->py, request->pz);
        response->success = true;
    }

    std::shared_ptr<RobotMover> robot_mover_;
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPose>::SharedPtr move_to_pose_service_;
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPosition>::SharedPtr move_to_position_service_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_moveit");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // Creating and initializing RobotMover and Service Node
  auto robot_mover = std::make_shared<RobotMover>(node);
  auto service_node = std::make_shared<RobotServiceNode>(robot_mover);

  // Execute robot pose print directly
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get current pose in main1.");
  robot_mover->printCurrentPose();

  // Adding service node to the executor
  executor.add_node(service_node);
  // Execute robot pose print directly
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get current pose in main2.");
  robot_mover->printCurrentPose();

  // Start the executor in a separate thread
  std::thread executor_thread([&executor]() { executor.spin(); });

  // Execute robot pose print directly
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get current pose in main3.");
  robot_mover->printCurrentPose();

  // Do not cancel the executor to keep the node alive for service requests
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
