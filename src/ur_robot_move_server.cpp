#include "rclcpp/rclcpp.hpp"
#include "ur5_robot_gripper/srv/move_to_pose.hpp"
#include "ur5_robot_gripper/srv/move_to_position.hpp"
#include "ur5_robot_gripper/robot_control.hpp"
#include <thread>

class RobotServiceNode : public rclcpp::Node
{
public:
    RobotServiceNode()
    : Node("robot_service_node")
    {
        // 初始化服务
        move_to_pose_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPose>(
            "move_to_pose", std::bind(&RobotServiceNode::moveToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

        move_to_position_service_ = this->create_service<ur5_robot_gripper::srv::MoveToPosition>(
            "move_to_position", std::bind(&RobotServiceNode::moveToPositionCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "RobotServiceNode has started and is ready.");
    }

    void initializeRobotMover()
    {
        // 在构造函数外部初始化 RobotMover
        robot_mover_ = std::make_shared<RobotMover>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "RobotMover initialized.");
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

    rclcpp::Service<ur5_robot_gripper::srv::MoveToPose>::SharedPtr move_to_pose_service_;
    rclcpp::Service<ur5_robot_gripper::srv::MoveToPosition>::SharedPtr move_to_position_service_;

    std::shared_ptr<RobotMover> robot_mover_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing RobotServiceNode...");

    // 创建节点
    auto node = std::make_shared<RobotServiceNode>();
    node->initializeRobotMover();  // 在节点初始化后再初始化 RobotMover

    // 创建并启动多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 在单独的线程中启动执行器
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 等待执行器线程结束
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}
