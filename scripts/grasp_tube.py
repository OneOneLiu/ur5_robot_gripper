#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from ur5_robot_gripper.action import MoveToPositionAction, MoveToPoseAction, MoveGripperAction, ExecuteGrasp
from rclpy.action import ActionClient, ActionServer
import time
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException

# 备注：
# 这个程序相对于原始的抓取管子程序进行了以下升级：
# 1. 代码模块化：将订阅姿态信息和执行抓取操作分离成了两个独立的类（PoseSubscriber 和 GraspExecutor），提高了代码的可读性和可维护性。
# 2. 数据存储和延迟执行：将接收到的姿态消息存储在 GraspExecutor 类中，并允许在需要时手动调用 execute_grasp() 方法执行抓取动作，而不是在接收到姿态消息时立即执行抓取。这种方式提高了系统的灵活性，避免了不必要的实时调用。
# 3. 简化的动作执行流程：将抓取动作的多个步骤（如预抓取、夹取）整合为一个单一的动作序列执行函数 execute_action_sequence()，减少了代码冗余。
# 4. 通用回调处理：统一了动作请求的回调处理，通过 generic_goal_response_callback() 和 get_result_callback() 简化了回调逻辑，减少了重复代码。

def construct_pose(position, quaternion):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = quaternion[0]
    pose.orientation.x = quaternion[1]
    pose.orientation.y = quaternion[2]
    pose.orientation.z = quaternion[3]
    return pose

class PoseSubscriber(Node):
    def __init__(self, grasp_executor):
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

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')
        self.get_logger().info('GraspExecutor node initialized.')

        # 创建action client
        self._pose_client = ActionClient(self, MoveToPoseAction, 'move_to_pose_action')
        self._gripper_client = ActionClient(self, MoveGripperAction, 'move_gripper_action')
        
        # 创建一个tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建一个action server，用于手动触发抓取某个tube的流程
        self._grasp_action_server = ActionServer(
            self,
            ExecuteGrasp,
            'execute_grasp_action',
            self.execute_grasp_callback
        )
        self.get_logger().info('GraspExecutor action server ready for grasping trigger.')
        
        # 存储姿态数组
        self.pose_array = None

    def set_pose_array(self, pose_array):
        self.pose_array = pose_array
        self.get_logger().debug('Pose array updated.')
    
    def execute_grasp_callback(self, goal_handle):
        self.get_logger().info(f'Executing grasp action for tube index {goal_handle.request.tube_index}...')
        result = ExecuteGrasp.Result()

        if self.pose_array is None or goal_handle.request.tube_index >= len(self.pose_array.poses):
            self.get_logger().warn('Invalid tube index or no pose array available for grasping.')
            result.success = False
            goal_handle.abort()
            return result

        self.execute_grasp(goal_handle.request.tube_index)
        goal_handle.succeed()

        # 动作成功完成，设置 success 为 True
        result.success = True
        return result

    def execute_grasp(self, tube_index):
        if self.pose_array is None:
            self.get_logger().warn('No pose array available for grasping.')
            return

        self.get_logger().info(f'Processing tube {tube_index}...')
        pre_grasp_pose, grasp_pose = self.calculate_grasping_poses(self.pose_array.poses[tube_index])

        transformed_pre_grasp_pose = self.transform_pose(pre_grasp_pose, 'isaac_world', 'world')
        transformed_grasp_pose = self.transform_pose(grasp_pose, 'isaac_world', 'world')

        # 执行抓取动作
        self.execute_action_sequence(transformed_pre_grasp_pose, transformed_grasp_pose)

    def calculate_grasping_poses(self, pose):
        # 实现与原代码相似的逻辑来计算pre-grasp和grasp的位姿
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        rotation_matrix = quat2mat(q)
        cylinder_y_axis = rotation_matrix[:, 1]
        up_direction = np.array([0, 0, 1])
        vector_horiz = np.cross(cylinder_y_axis, up_direction)
        vector_horiz = vector_horiz / np.linalg.norm(vector_horiz)
        result_vector = np.cross(vector_horiz, cylinder_y_axis)
        result_vector = result_vector / np.linalg.norm(result_vector)
        opposite_vector = -result_vector
        rotation_matrix_opposite = np.column_stack((vector_horiz, -cylinder_y_axis, opposite_vector))
        quaternion_opposite = mat2quat(rotation_matrix_opposite)
        
        cylinder_center = np.array([pose.position.x, pose.position.y, pose.position.z])
        pre_grasp_point = cylinder_center + result_vector * 0.1 + cylinder_y_axis * 0.03
        grasp_point = cylinder_center + result_vector * 0.005 + cylinder_y_axis * 0.03
        
        pre_grasp_pose = construct_pose(pre_grasp_point, quaternion_opposite)
        grasp_pose = construct_pose(grasp_point, quaternion_opposite)
        
        return pre_grasp_pose, grasp_pose

    def transform_pose(self, pose, source_frame, target_frame):
        while True:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                break
            except TransformException as ex:
                self.get_logger().debug(f'Could not transform {source_frame} to {target_frame}: {ex}')
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, t)
        return transformed_pose

    def execute_action_sequence(self, pre_grasp_pose, grasp_pose):
        # Consolidate sending goals and handling callbacks into a single function
        self.send_pose_goal(pre_grasp_pose)
        while not self.robot_ready:
            time.sleep(0.1)
        self.send_gripper_goal(0.5)  # 关闭夹爪
        while not self.gripper_ready:
            time.sleep(0.1)
        self.send_pose_goal(grasp_pose)
        while not self.robot_ready:
            time.sleep(0.1)
        self.send_gripper_goal(0.7)  # 关闭夹爪
        while not self.gripper_ready:
            time.sleep(0.1)
        self.send_pose_goal(pre_grasp_pose)
        while not self.robot_ready:
            time.sleep(0.1)

    def send_pose_goal(self, pose):
        goal_msg = MoveToPoseAction.Goal()
        goal_msg.px = pose.position.x
        goal_msg.py = pose.position.y
        goal_msg.pz = pose.position.z
        goal_msg.qw = pose.orientation.w
        goal_msg.qx = pose.orientation.x
        goal_msg.qy = pose.orientation.y
        goal_msg.qz = pose.orientation.z
        
        self.robot_ready = False  # 设置机器人状态为不准备好
        self._pose_client.wait_for_server()
        self._send_goal_future = self._pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda future: self.generic_goal_response_callback(future, action_type="robot"))

    def send_gripper_goal(self, target_position):
        goal_msg = MoveGripperAction.Goal()
        goal_msg.target_position = target_position
        self.gripper_ready = False  # 设置夹爪状态为不准备好
        self._gripper_client.wait_for_server()
        self._send_gripper_goal_future = self._gripper_client.send_goal_async(goal_msg)
        self._send_gripper_goal_future.add_done_callback(lambda future: self.generic_goal_response_callback(future, action_type="gripper"))

    def generic_goal_response_callback(self, future, action_type="robot"):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, action_type))

    def get_result_callback(self, future, action_type="robot"):
        result = future.result().result
        if result.success:
            self.get_logger().info('Action succeeded!')
            if action_type == "gripper":
                self.gripper_ready = True  # 设置夹爪状态为准备好
            elif action_type == "robot":
                self.robot_ready = True  # 设置机器人状态为准备好
        else:
            self.get_logger().info('Action failed!')
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('main').info('Initializing ROS...')

    grasp_executor = GraspExecutor()
    pose_subscriber = PoseSubscriber(grasp_executor)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(grasp_executor)
    executor.add_node(pose_subscriber)

    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()