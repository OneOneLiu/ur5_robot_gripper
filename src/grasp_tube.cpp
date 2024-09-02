#include "ur5_robot_gripper/robot_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

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
        if (msg->poses.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Received PoseArray has less than 10 poses.");
            return;
        }

        // 获取第 10 个姿态
        auto target_pose = msg->poses[9];

        // 调用 RobotMover 类的 moveToPosition 方法移动到目标位置
        RCLCPP_INFO(this->get_logger(), "Moving to the 10th pose...");
        // 打印第 10 个姿态的位置和方向
        RCLCPP_INFO(this->get_logger(), "10th Pose Position: x=%.3f, y=%.3f, z=%.3f", 
                    target_pose.position.x, 
                    target_pose.position.y, 
                    target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "10th Pose Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                    target_pose.orientation.x, 
                    target_pose.orientation.y, 
                    target_pose.orientation.z, 
                    target_pose.orientation.w);
        // TODO: 写一个TF转换方法，现在这个目标位置是手动调整的
        // 写一个笛卡尔坐标系移动方法，现在这样移动姿态有些扭曲
        // robot_mover_.moveToCartesianPosition(
        //     target_pose.position.y,
        //     -target_pose.position.x,
        //     target_pose.position.z + 0.2
        // );
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    RobotMover robot_mover_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // 创建节点并开始订阅
    auto node = std::make_shared<rclcpp::Node>("robot_moveit");
    auto subscriber_node = std::make_shared<PoseSubscriber>(node);
    RobotMover robot_mover(node);
    robot_mover.moveToCartesianPosition(0.08, -0.58, 0.3);
    // 运行节点，直到被手动停止
    rclcpp::spin(subscriber_node);

    // 释放节点资源，确保没有未完成的任务
    rclcpp::shutdown();

    return 0;
}
