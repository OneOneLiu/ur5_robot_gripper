#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ur5_robot_gripper.srv import PoseTransform
import time

# action client
from rclpy.action import ActionClient
from ur5_robot_gripper.action import MoveToPositionAction
from ur5_robot_gripper.action import MoveToPoseAction
from ur5_robot_gripper.action import MoveGripperAction
# tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat

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
        self.pre_action = "grasp"
        
        # 创建一个action client
        self._pose_client = ActionClient(self, MoveToPoseAction, 'move_to_pose_action')
        self.pose_action_done = True
        
        # Create a gripper action client  <-- Add Gripper Action Client
        self._gripper_client = ActionClient(self, MoveGripperAction, 'move_gripper_action')
        self.gripper_action_done = True
        
        self.tube_index = 0

        # 创建一个tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg):
        # 如果姿态数组小于14，记录警告信息
        # if len(msg.poses) < 14:
        #     self.get_logger().warn('Received PoseArray has less than 14 poses.')
        #     return 0
        self.pose_msg = msg
        self.get_logger().debug('Received and saved tube75 poses.')
        
        # 如果动作已经完成，执行下一个动作
        if self.pose_action_done:
            rclpy.logging.get_logger('pose_callback').warn('action done, action server is ready for next goal')
            # Calculate the pre-grasp and grasp poses
            pre_grasp_point, grasping_point, quaternion_opposite = self.cal_grasping_pose(self.pose_msg.poses[self.tube_index])
                
            pre_grasp_pose = self.construct_pose(pre_grasp_point, quaternion_opposite)
            grasping_pose = self.construct_pose(grasping_point, quaternion_opposite)
            
            transformed_pre_grasp_pose = self.transform_pose(pre_grasp_pose, 'isaac_world', 'world')
            transformed_grasp_pose = self.transform_pose(grasping_pose, 'isaac_world', 'world')
            if self.pre_action == "grasp":
                self.send_gripper_goal(0.5)
                self.send_pose_goal((transformed_pre_grasp_pose.position.x, transformed_pre_grasp_pose.position.y, transformed_pre_grasp_pose.position.z), (transformed_pre_grasp_pose.orientation.w, transformed_pre_grasp_pose.orientation.x, transformed_pre_grasp_pose.orientation.y, transformed_pre_grasp_pose.orientation.z))
                rclpy.logging.get_logger('pose_callback').warn('Finished sending pre-grasp goal')
                self.pose_action_done = False
                self.pre_action = "pre_grasp"
                rclpy.logging.get_logger('pose_callback').warn('action is not done, waiting for action server to be ready')
            else:
                self.send_pose_goal((transformed_grasp_pose.position.x, transformed_grasp_pose.position.y, transformed_grasp_pose.position.z), (transformed_grasp_pose.orientation.w, transformed_grasp_pose.orientation.x, transformed_grasp_pose.orientation.y, transformed_grasp_pose.orientation.z))
                rclpy.logging.get_logger('pose_callback').warn('Finished sending grasp goal')
                self.pose_action_done = False
                self.pre_action = "grasp"
                # 只有当两个动作都完成时，才能更新tube index到下一个动作
                self.tube_index += 1
                if self.tube_index == len(self.pose_msg.poses):
                    self.tube_index = 0
                rclpy.logging.get_logger('pose_callback').warn('action is not done, waiting for action server to be ready')
        else:
            # rclpy.logging.get_logger('pose_callback').warn('action is not done, waiting for action server to be ready')
            # sleep for 0.5 seconds
            # time.sleep(0.5)
            pass
        return 0

    def construct_pose(self, position, quaternion):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        
        return pose

    def cal_grasping_pose(self, pose, visual=False):
        # calculate orientation
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        # 打印圆柱体中心位置
        self.get_logger().warn("第{}个圆柱体".format(self.tube_index))
        # self.get_logger().warn("圆柱体中心位置: {},{},{}".format(pose.position.x, pose.position.y, pose.position.z))
        # 将四元数转换为旋转矩阵
        rotation_matrix = quat2mat(q)
        # 提取旋转矩阵的第三列作为圆柱体的Z轴
        cylinder_y_axis = rotation_matrix[:, 1]
        # 打印圆柱体Z轴方向
        # self.get_logger().warn("圆柱体Y轴方向: {}, {}, {}".format(cylinder_y_axis[0], cylinder_y_axis[1], cylinder_y_axis[2]))
        
        # 定义向上方向（假设沿着世界坐标系的Z轴）
        up_direction = np.array([0, 0, 1])

        # 第一次叉乘，得到与Z轴和向上方向垂直的水平向量
        vector_horiz = np.cross(cylinder_y_axis, up_direction)

        # 检查是否为零向量
        if np.linalg.norm(vector_horiz) == 0:
            # 如果为零，说明两个向量平行，选择另一个方向，例如X轴 # TODO: 思考为什么选择X轴
            up_direction = np.array([1, 0, 0])
            vector_horiz = np.cross(cylinder_y_axis, up_direction)

        # 归一化水平向量
        vector_horiz = vector_horiz / np.linalg.norm(vector_horiz)
        # self.get_logger().warn("水平向量:{}, {}, {}".format(vector_horiz[0], vector_horiz[1], vector_horiz[2]))

        # 第二次叉乘，计算朝上边线的方向
        result_vector = np.cross(vector_horiz, cylinder_y_axis)

        # 归一化结果向量
        result_vector = result_vector / np.linalg.norm(result_vector)
        # self.get_logger().warn("朝上边线方向:{}, {}, {}".format(result_vector[0], result_vector[1], result_vector[2]))
        
        # 取朝上边线方向的反方向向量
        opposite_vector = -result_vector
        # self.get_logger().warn("朝上边线反方向:{}, {}, {}".format(opposite_vector[0], opposite_vector[1], opposite_vector[2]))
        # 将旋转矩阵转换为四元数
        # 创建旋转矩阵，使用水平向量、圆柱体Y轴和反方向向量
        rotation_matrix_opposite = np.column_stack((vector_horiz, -cylinder_y_axis, opposite_vector))
        quaternion_opposite = mat2quat(rotation_matrix_opposite)
        # self.get_logger().warn("朝上边线反方向的四元数表示: w={}, x={}, y={}, z={}".format(
            # quaternion_opposite[0], quaternion_opposite[1], quaternion_opposite[2], quaternion_opposite[3]))
        
        # 获取圆柱体中心位置
        cylinder_center = np.array([pose.position.x, 
                                    pose.position.y, 
                                    pose.position.z])

        # 朝上边线方向的单位向量已经是 result_vector，因此我们可以直接乘以 0.05 得到新的点的位置偏移
        offset = result_vector * 0.1 + cylinder_y_axis * 0.03
        # 计算新的点的坐标
        pre_grasp_point = cylinder_center + offset

        # 抓取圆柱体中心
        offset = result_vector * 0.005 + cylinder_y_axis * 0.03
        # 计算新的点的坐标
        grasp_point = cylinder_center + offset

        # 打印结果
        # self.get_logger().warn("沿着朝上边线方向0.05米的点坐标: {}, {}, {}".format(new_point[0], new_point[1], new_point[2]))
        
        return pre_grasp_point, grasp_point, quaternion_opposite

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
        # rclpy.logging.get_logger('send_action_goal').info('Found action server.')
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
            self.get_logger().info('Moveing Action Goal succeeded!')
            # For testing purposes, sleep for 2 seconds
            rclpy.logging.get_logger('get_result_callback').warn('Sleeping for 2 seconds...')
            time.sleep(2)
            self.action_done = True
    
    ## 以下是第二个动作的代码
    def send_pose_goal(self, position, quaternion):
        '''
        发送目标位置给action server
        args:
            position: 目标位置
                px: position[0]
                py: position[1]
                pz: position[2]
        '''
        goal_msg = MoveToPoseAction.Goal()
        goal_msg.px = position[0]
        goal_msg.py = position[1]
        goal_msg.pz = position[2]
        goal_msg.qw = quaternion[0]
        goal_msg.qx = quaternion[1]
        goal_msg.qy = quaternion[2]
        goal_msg.qz = quaternion[3]
        
        # Use the correct action client for pose
        self._pose_client.wait_for_server()  # <-- Use the correct action client here
        rclpy.logging.get_logger('send_action_goal').info('Found move to pose action server.')
        self._send_goal_future = self._pose_client.send_goal_async(goal_msg)  # <-- Use _pose_client here
        
        self._send_goal_future.add_done_callback(self.pose_goal_response_callback)

    def pose_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_pose_result_callback)

    def get_pose_result_callback(self, future):
        self.get_logger().info('Received Done signal :)')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        if result.success:
            self.get_logger().info('Goal succeeded!')
            # For testing purposes, sleep for 2 seconds
            rclpy.logging.get_logger('get_result_callback').warn('Sleeping for 2 seconds...')
            time.sleep(2)
            self.pose_action_done = True
 
    def transform_pose(self, pose, source_frame, target_frame):
        while True:
            # wait for transform buffer to be filled
            try:
                self.t = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time())
                # self.get_logger().info(
                #     f'Found transform {source_frame} to {target_frame})')
                break
            except TransformException as ex:
                self.get_logger().debug(
                    f'Could not transform {source_frame} to {target_frame}: {ex}')
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, self.t)
        self.get_logger().info('Pose: {0}'.format(pose))
        self.get_logger().info('Transformed pose: {0}'.format(transformed_pose))
        return transformed_pose
    
    def send_gripper_goal(self, target_position):
        '''
        Send goal to control the gripper
        args:
            target_position: Desired gripper position (e.g., 0.8 to close, 0.0 to open)
        '''
        goal_msg = MoveGripperAction.Goal()
        goal_msg.target_position = target_position
        
        self._gripper_client.wait_for_server()
        self.get_logger().info('Sending goal to gripper action server.')
        self._send_gripper_goal_future = self._gripper_client.send_goal_async(goal_msg)
        self._send_gripper_goal_future.add_done_callback(self.gripper_goal_response_callback)
        self.get_logger().info('Gripper goal sent.')

    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Gripper goal rejected :(')
            return

        self.get_logger().info('Gripper goal accepted :)')
        self._get_gripper_result_future = goal_handle.get_result_async()
        self._get_gripper_result_future.add_done_callback(self.get_gripper_result_callback)

    def get_gripper_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Gripper action succeeded!')
            self.gripper_action_done = True  # Mark the gripper action as done
        else:
            self.get_logger().info('Gripper action failed!')

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
