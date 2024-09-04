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
    }

private:
    void moveToPoseCallback(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Request> request,
                            std::shared_ptr<ur5_robot_gripper::srv::MoveToPose::Response> response)
    {
        robot_mover_->moveToPose(request->px, request->py, request->pz, request->qx, request->qy, request->qz, request->qw);
        response->success = true;
    }

    void moveToPositionCallback(const std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Request> request,
                                std::shared_ptr<ur5_robot_gripper::srv::MoveToPosition::Response> response)
    {
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
  RobotMover robot_mover(node);
  auto service_node = std::make_shared<RobotServiceNode>(std::make_shared<RobotMover>(node));

  // Adding service node to the executor
  executor.add_node(service_node);

  // Start the executor in a separate thread
  std::thread executor_thread([&executor]() { executor.spin(); });

  // Execute robot pose print directly
  robot_mover.printCurrentPose();

  // Give some time for the node to process
  rclcpp::sleep_for(std::chrono::seconds(1));

//   executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
