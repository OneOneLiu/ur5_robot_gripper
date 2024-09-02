#include "ur5_robot_gripper/robot_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <thread>

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber(const rclcpp::Node::SharedPtr& node)
    : Node("pose_subscriber"), robot_mover_(node)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        // 订阅 /tube75_poses 话题
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/tube75_poses", 10, std::bind(&PoseSubscriber::poseCallback, this, std::placeholders::_1));
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // 如果姿态数组小于14，记录警告信息
        if (msg->poses.size() < 14) {
            RCLCPP_WARN(this->get_logger(), "Received PoseArray has less than 14 poses.");
            return;
        }

        // 循环遍历从第1到第14个姿态
        for (int i = 0; i < 14; ++i) {
            auto target_pose = msg->poses[i];

            // 打印目标姿态的位置和方向
            RCLCPP_INFO(this->get_logger(), "Moving to pose %d...", i + 1);
            RCLCPP_INFO(this->get_logger(), "Pose %d Position: x=%.3f, y=%.3f, z=%.3f", 
                        i + 1,
                        target_pose.position.x, 
                        target_pose.position.y, 
                        target_pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Pose %d Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                        i + 1,
                        target_pose.orientation.x, 
                        target_pose.orientation.y, 
                        target_pose.orientation.z, 
                        target_pose.orientation.w);

            // 使用笛卡尔路径移动到指定位置
            robot_mover_.moveToCartesianPosition(
                target_pose.position.y,
                -target_pose.position.x,
                target_pose.position.z + 0.05
            );

            // 可以选择在每次移动后进行一个小的延时，以确保运动稳定
            rclcpp::sleep_for(std::chrono::seconds(2)); // 根据需要调整时间
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    RobotMover robot_mover_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "robot_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // 创建并启动多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 创建 PoseSubscriber 实例
    auto subscriber_node = std::make_shared<PoseSubscriber>(node);
    executor.add_node(subscriber_node);

    // 在单独的线程中启动执行器
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 等待执行器线程结束
    executor_thread.join();

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}