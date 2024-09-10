#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ur5_robot_gripper.srv import PoseTransform
import time

# action client
from rclpy.action import ActionClient
from ur5_robot_gripper.action import MoveToPositionAction

# tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import numpy as np
from transforms3d.quaternions import quat2mat

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
        
        # 创建一个action client
        self._action_client = ActionClient(self, MoveToPositionAction, 'move_to_position_action')
        self.action_done = True
        self.tube_index = 0

        # 创建一个tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg):
        # 如果姿态数组小于14，记录警告信息
        if len(msg.poses) < 14:
            self.get_logger().warn('Received PoseArray has less than 14 poses.')
            return 0
        self.pose_msg = msg
        self.get_logger().debug('Received and saved tube75 poses.')
        
        # 如果动作已经完成，执行下一个动作
        if self.action_done:
            transformed_pose = self.transform_pose(self.pose_msg.poses[self.tube_index], 'isaac_world', 'base_link')
            # calculate orientation
            q = [self.pose_msg.poses[self.tube_index].orientation.w, self.pose_msg.poses[self.tube_index].orientation.x, self.pose_msg.poses[self.tube_index].orientation.y, self.pose_msg.poses[self.tube_index].orientation.z]
            # 打印圆柱体中心位置
            self.get_logger().warn("圆柱体中心位置: {},{},{}".format(self.pose_msg.poses[self.tube_index].position.x, self.pose_msg.poses[self.tube_index].position.y, self.pose_msg.poses[self.tube_index].position.z))
            # 将四元数转换为旋转矩阵
            rotation_matrix = quat2mat(q)
            # 提取旋转矩阵的第三列作为圆柱体的Z轴
            cylinder_z_axis = rotation_matrix[:, 2]
            # 打印圆柱体Z轴方向
            self.get_logger().warn("圆柱体Z轴方向: {}, {}, {}".format(cylinder_z_axis[0], cylinder_z_axis[1], cylinder_z_axis[2]))
            
            # 定义向上方向（假设沿着世界坐标系的Z轴）
            up_direction = np.array([0, 0, 1])

            # 第一次叉乘，得到与Z轴和向上方向垂直的水平向量
            vector_horiz = np.cross(cylinder_z_axis, up_direction)

            # 检查是否为零向量
            if np.linalg.norm(vector_horiz) == 0:
                # 如果为零，说明两个向量平行，选择另一个方向，例如X轴 # TODO: 思考为什么选择X轴
                up_direction = np.array([1, 0, 0])
                vector_horiz = np.cross(cylinder_z_axis, up_direction)

            # 归一化水平向量
            vector_horiz = vector_horiz / np.linalg.norm(vector_horiz)
            self.get_logger().warn("水平向量:{}, {}, {}".format(vector_horiz[0], vector_horiz[1], vector_horiz[2]))

            # 第二次叉乘，计算朝上边线的方向
            result_vector = np.cross(vector_horiz, cylinder_z_axis)

            # 归一化结果向量
            result_vector = result_vector / np.linalg.norm(result_vector)
            self.get_logger().warn("朝上边线方向:{}, {}, {}".format(result_vector[0], result_vector[1], result_vector[2]))

            self.send_action_goal([transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z+0.2])
            self.action_done = False
            self.tube_index += 1
            if self.tube_index == 14:
                self.tube_index = 0

            return 0

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
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Received Done signal :)')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        if result.success:
            self.get_logger().info('Goal succeeded!')
            self.action_done = True
    
    def transform_pose(self, pose, source_frame, target_frame):
        while True:
            # wait for transform buffer to be filled
            try:
                self.t = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time())
                self.get_logger().info(
                    f'Found transform {source_frame} to {target_frame})')
                break
            except TransformException as ex:
                self.get_logger().debug(
                    f'Could not transform {source_frame} to {target_frame}: {ex}')
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, self.t)
        self.get_logger().info('Pose: {0}'.format(pose))
        self.get_logger().info('Transformed pose: {0}'.format(transformed_pose))
        return transformed_pose      

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('main').info('Initializing ROS...')

    # 创建主节点
    node = PoseSubscriber()
    rclpy.logging.get_logger('main').info('Main node created.')

    # 创建并启动多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()
    
    # Send one action request test
    # rclpy.spin(node)

    # rclpy.logging.get_logger('main').info('Shutting down ROS...')

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
