#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ur5_robot_gripper.srv import PoseTransform
import time

from rclpy.action import ActionClient
from ur5_robot_gripper.action import MoveToPositionAction

class PoseSubscriber(Node):
    def __init__(self):
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
        
        # 
        self._action_client = ActionClient(self, MoveToPositionAction, 'move_to_position_action')

    def pose_callback(self, msg):
        self.get_logger().info('Received PoseArray message.')

        # 如果姿态数组小于14，记录警告信息
        if len(msg.poses) < 14:
            self.get_logger().warn('Received PoseArray has less than 14 poses.')
            return 0
        self.pose_msg = msg
        self.get_logger().info('Received and saved tube75 poses.')

    def send_action_goal(self, position):
        '''
        发送目标位置给action server
        args:
            position: 目标位置
                px: position[0]
                py: position[1]
                pz: position[2]
        '''
        goal_msg = MoveToPositionAction.Goal()
        goal_msg.px = position[0]
        goal_msg.py = position[1]
        goal_msg.pz = position[2]
        
        self._action_client.wait_for_server()
        rclpy.logging.get_logger('send_action_goal').info('Found action server.')
        
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('main').info('Initializing ROS...')

    # 创建主节点
    node = PoseSubscriber()
    rclpy.logging.get_logger('main').info('Main node created.')

    # 创建并启动多线程执行器
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)

    # executor.spin()
    
    # Send one action request test
    future = node.send_action_goal([0.1, 0.1, 0.27])
    rclpy.logging.get_logger('main').info('Sent action goal.')
    rclpy.spin_until_future_complete(node, future)

    rclpy.logging.get_logger('main').info('Shutting down ROS...')

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
