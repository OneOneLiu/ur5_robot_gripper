#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

'''
PoseSubscriber 类是一个 ROS 2 节点，负责订阅 /tube75_poses 话题，该话题发布的是一组tube的位置和姿态信息（PoseArray）。当该节点接收到姿态消息后，会将这些姿态信息传递给 GraspExecutor 类进行后续的抓取处理。
'''
class PoseSubscriber(Node):
    
    def __init__(self, grasp_executor):
        '''
        初始化 PoseSubscriber 节点，并建立对 /tube75_poses 话题的订阅。
        args: grasp_executor: GraspExecutor 类的实例，它负责处理接收到的姿态数据并执行抓取任务。
        '''
        super().__init__('pose_subscriber')
        self.get_logger().info('PoseSubscriber node initialized.')

        # 创建一个subscriber 订阅 /tube75_poses 话题
        self.subscription = self.create_subscription(
            PoseArray,
            '/tube75_poses',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /tube75_poses topic.')
        
        # 将抓取执行器实例保存为属性
        self.grasp_executor = grasp_executor
        
        self.robot_ready = True
        self.gripper_ready = True

    def pose_callback(self, msg):
        self.get_logger().debug('Received and saved tube75 poses.')
        # 将接收到的pose消息保存到抓取执行器中
        self.grasp_executor.set_pose_array(msg)