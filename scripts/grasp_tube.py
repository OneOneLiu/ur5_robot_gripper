#!/usr/bin/env python3
import time
import numpy as np
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from rclpy.action import ActionClient, ActionServer
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.axangles import axangle2mat
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException

from ur5_robot_gripper.action import MoveToPositionAction, MoveToPoseAction, MoveGripperAction, ExecuteGrasp, MoveToJointPosition
from ur5_robot_gripper.srv import PrintPose
from utils import construct_pose, pose_to_matrix, apply_transform, quaternion_difference
from tube_pose_sub import PoseSubscriber

# 备注：
# 这个程序相对于原始的抓取管子程序进行了以下升级：
# 1. 代码模块化：将订阅姿态信息和执行抓取操作分离成了两个独立的类（PoseSubscriber 和 GraspExecutor），提高了代码的可读性和可维护性。
# 2. 数据存储和延迟执行：将接收到的姿态消息存储在 GraspExecutor 类中，并允许在需要时手动调用 execute_grasp() 方法执行抓取动作，而不是在接收到姿态消息时立即执行抓取。这种方式提高了系统的灵活性，避免了不必要的实时调用。
# 3. 简化的动作执行流程：将抓取动作的多个步骤（如预抓取、夹取）整合为一个单一的动作序列执行函数 execute_action_sequence()，减少了代码冗余。
# 4. 通用回调处理：统一了动作请求的回调处理，通过 generic_goal_response_callback() 和 get_result_callback() 简化了回调逻辑，减少了重复代码。

'''
ToDos:
- 机器人规划最短路径
    - 路径成本的评价
    - 等价路径的判断
- 最快找到可行的x, z姿态
- 监测tube是否在机器人上
- fine tune tube放置角度
- 放置的高度需要改一下，不然会碰到已经放进去的tube
BUG:
- （Solved）第二次抓取时，tube姿态变成横着的了，没有变成直立： 由于机器人的位姿没有更新导致的
'''

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')
        self.get_logger().info('GraspExecutor node initialized.')

        # 创建action client: 移动到指定位姿，夹取动作，移动到指定关节角度
        self._pose_client = ActionClient(self, MoveToPoseAction, 'move_to_pose_action')
        self._gripper_client = ActionClient(self, MoveGripperAction, 'move_gripper_action')
        self._joint_position_client = ActionClient(self, MoveToJointPosition, 'move_to_joint_position_action')  # Added Action Client for MoveToJointPosition
        
        # 创建一个service client，用于获取当前机器人的位姿
        self._state_client = self.create_client(PrintPose, 'print_current_pose')
        while not self._state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for print_current_pose service to become available...')
        
        # 创建一个tf2 listener，用于坐标变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建抓取tube的 action server，用于手动触发抓取某个tube的流程 （主要循环功能）
        self._grasp_action_server = ActionServer(
            self,
            ExecuteGrasp,
            'execute_grasp_action',
            self.execute_grasp_callback
        )
        self.get_logger().info('GraspExecutor action server ready for grasping trigger.')
        
        # 存储姿态数组
        self.pose_array = None
        # 存储当前机器人的位姿和关节角度
        self.current_joint_angles = None
        
        # 初始化机器人和夹爪的状态
        self.robot_ready = True
        self.gripper_ready = True
        self.action_succeeded = True
        
        # 初始化rack上的占用情况
        self.rack_occupancy = np.zeros((6, 15))  # 6x15的矩阵，表示rack上每个位置的占用情况

    def execute_grasp_callback(self, goal_handle):
        '''
        _grasp_action_server请求的回调函数，用于执行抓取某个tube的动作
        args: goal_handle: action 提供的 goal_handle，包含了抓取任务请求的所有相关信息。本请求中具体包括：
            goal_handle.request.tube_index：表示要抓取的管子的索引（整数类型）
        '''
        self.get_logger().info(f'Executing grasp action for tube index {goal_handle.request.tube_index}...')
        result = ExecuteGrasp.Result()

        if self.pose_array is None or goal_handle.request.tube_index >= len(self.pose_array.poses):
            self.get_logger().warn('Invalid tube index or no pose array available for grasping.')
            result.success = False
            goal_handle.abort()
            return result
        
        # 调用抓取函数执行抓取动作
        self.execute_grasp(goal_handle.request.tube_index)
        goal_handle.succeed()
        
        # 动作成功完成，设置 success 为 True
        result.success = True
        
        return result

    def execute_grasp(self, tube_index):
        '''
        执行具体的抓取操作的函数：根据tube的索引 (tube_index) 计算抓取姿态，接着将这些姿态转换到机器人坐标系中，最终调用动作序列执行函数，执行抓取任务。
        '''
        if self.pose_array is None:
            self.get_logger().warn('No pose array available for grasping.')
            return

        self.get_logger().info(f'Processing tube {tube_index}...')

        # 计算预抓取和抓取姿态
        pre_grasp_pose, grasp_pose = self.calculate_grasping_poses(self.pose_array.poses[tube_index])
        
        # 将姿态转换到机器人坐标系中
        transformed_pre_grasp_pose = self.transform_pose(pre_grasp_pose, 'isaac_world', 'world')
        transformed_grasp_pose = self.transform_pose(grasp_pose, 'isaac_world', 'world')

        # 执行抓取动作
        self.execute_action_sequence(transformed_pre_grasp_pose, transformed_grasp_pose, tube_index)

    def execute_action_sequence(self, pre_grasp_pose, grasp_pose, tube_index):
        # Grasp action sequence: Move to pre-grasp pose -> Close gripper -> Move to grasp pose -> Close gripper -> Move to pre-grasp pose
        self.get_logger().info('Executing action sequence...')
        self.send_pose_goal(pre_grasp_pose, velocity_scaling=0.07)
        self.get_logger().info('Pre-grasp pose reached.')
        self.send_gripper_goal(0.5)  # 关闭夹爪
        self.get_logger().info('Gripper pre-closed.')
        self.send_pose_goal(grasp_pose, velocity_scaling=0.01)
        self.get_logger().info('Grasp pose reached.')
        self.send_gripper_goal(0.7)  # 关闭夹爪
        self.get_logger().info('Tube grasped.')
        self.send_pose_goal(pre_grasp_pose, velocity_scaling=0.02)
        self.get_logger().info('Pre-grasp pose reached.')
        
        # Add placing action here
        target_placing_pose, placing_poses = self.calculate_placing_pose(tube_index)
        self.placing_tube(target_placing_pose, placing_poses, tube_index)
        # Return to pre-grasp pose
        self.send_pose_goal(pre_grasp_pose, velocity_scaling=0.02)
        
    def calculate_placing_pose(self, tube_index):
        # 依据tube的当前姿态以及放置位置的可达性，实时计算放置tube的姿态
        
        ###################################
        # 1. 一些预设
        ###################################
        # 设定放置管子的位置，姿态在后面生成
        ## 根据rack上的位置占用情况，先找到一个第一个空的位置
        hole_x, hole_y, hole_z = self.check_rack_avalaibility()
        target_placing_pose = Pose()
        target_placing_pose.position.x = hole_x
        target_placing_pose.position.y = hole_y
        target_placing_pose.position.z = hole_z + 0.05 # 设定位置为孔的第一个位置上方5cm
        
        # target_placing_pose = Pose()
        # target_placing_pose.position.x = 0.60
        # target_placing_pose.position.y = 0.32
        # target_placing_pose.position.z = 0.08 + 0.05 # 设定位置为孔的第一个位置上方5cm
        
        # 获取当前管子的姿态并转换成旋转矩阵
        current_tube_pose = self.pose_array.poses[tube_index]
        current_tube_rotation_matrix = quat2mat([current_tube_pose.orientation.w,
                                            current_tube_pose.orientation.x,
                                            current_tube_pose.orientation.y,
                                            current_tube_pose.orientation.z])
        
        # 提取当前管子的 y 轴方向，并设定目标方向为cap向上直立（与世界z轴重合）
        current_y_axis = current_tube_rotation_matrix[:, 1]  # 当前的 y 轴方向
        target_y_axis = np.array([0, 0, 1])  # 目标 y 轴是世界坐标系中的 z 轴
        
        ###################################
        # 2. 计算如何进行y轴旋转
        ###################################
        # 通过轴角方式计算将当前 y 轴旋转到目标 y 轴 (世界坐标系z轴) 的旋转矩阵
        
        # 先计算旋转轴
        rotation_axis = np.cross(current_y_axis, target_y_axis)
        if np.linalg.norm(rotation_axis) > 1e-6:  # 避免零向量的情况
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            # 再计算旋转角度
            rotation_angle = np.arccos(np.dot(current_y_axis, target_y_axis))
            rotation_matrix_to_target_y = axangle2mat(rotation_axis, rotation_angle)
        else:
            # 如果 y 轴已经对齐，不需要旋转
            rotation_matrix_to_target_y = np.eye(3)
        
        # 将当前表示姿态的旋转矩阵与上面求到的旋转矩阵相乘 -> 得到的就是立着的管子的旋转矩阵（姿态）
        tube_y_2_world_z_rotation_matrix = np.dot(rotation_matrix_to_target_y, current_tube_rotation_matrix) # np.dot(R1, R2)：这表示先执行 R2 的旋转，然后执行 R1 的旋转。
        
        ###################################
        # 3. 生成x, z轴候选
        ###################################
        # x,z 轴没有特别要求，所以生成一组候选的旋转矩阵，后续从中实验选择
        
        # 创建一个数组存储所有候选的旋转四元数
        candidate_quaternions = []
        ## 下面需要生成一组候选的，具有不同的x,z轴旋转的旋转矩阵
        for angle_deg in range(0, 360, 10):
            angle_rad = np.radians(angle_deg)
            rotation_matrix = axangle2mat([0, 0, 1], angle_rad)
            
            # 最终的试管目标姿态矩阵即为：上面求到的保证tube y轴与世界z轴对齐，同时乘上不同的绕世界z的旋转矩阵
            final_rotation_matrix = np.dot(rotation_matrix, tube_y_2_world_z_rotation_matrix) 
            final_quaternion = mat2quat(final_rotation_matrix)
            candidate_quaternions.append(final_quaternion)
        
        ###################################
        # 4. 求解机器人姿态
        ###################################
        # 前述三步求得的是tube的姿态，下面将其转变为控制机器人所需的姿态
        
        # 获取当前机器人的位姿
        self.get_current_robot_state()
        time.sleep(1)
        robot_pose = self.current_robot_pose
        robot_pose_in_isaac_world = self.transform_pose(robot_pose, 'world', 'isaac_world')

        # 计算tube与机器人之间的变换矩阵 （假设它在后续过程中都保持不变）
        robot_matrix = pose_to_matrix(robot_pose_in_isaac_world)
        tube_matrix = pose_to_matrix(current_tube_pose)
        
        # 计算robot-tube相对变换矩阵
        relative_transform = np.linalg.inv(robot_matrix).dot(tube_matrix)
        
        # 设置一个有倾向的四元数，用于后续计算相似度
        d_x=0.5178879634425592
        d_y=-0.4814145813368711
        d_z=0.5179544730306757
        d_w=-0.48140961983022185
        desired_quaternion = [d_w, d_x, d_y, d_z]
        q_differences = []
        # 生成机器人姿态候选
        transformed_placing_poses = []
        for candidate_quaternion in candidate_quaternions:
            target_placing_pose.orientation.w = candidate_quaternion[0]
            target_placing_pose.orientation.x = candidate_quaternion[1]
            target_placing_pose.orientation.y = candidate_quaternion[2]
            target_placing_pose.orientation.z = candidate_quaternion[3]
            new_robot_pose = apply_transform(target_placing_pose, relative_transform)
            
            transformed_placing_pose = self.transform_pose(new_robot_pose, 'isaac_world', 'world')
            transformed_placing_poses.append(transformed_placing_pose)

            q_difference = quaternion_difference(desired_quaternion, [transformed_placing_pose.orientation.w, transformed_placing_pose.orientation.x, transformed_placing_pose.orientation.y, transformed_placing_pose.orientation.z])
            q_differences.append(abs(q_difference))
            
            self.get_logger().warning(f'The q_difference is {q_difference}.')
        
        # 按照相似度排序（q_difference 值越小越靠前）
        sorted_indices = sorted(range(len(q_differences)), key=lambda i: q_differences[i])

        # 根据排序后的索引重新排列姿态和四元数
        sorted_transformed_placing_poses = [transformed_placing_poses[i] for i in sorted_indices]
        
        return target_placing_pose, sorted_transformed_placing_poses
            
    def placing_tube(self, target_placing_pose, placing_poses, tube_index):
        # 依次尝试每个候选的旋转四元数
        for i, placing_pose in enumerate(placing_poses):
            self.get_logger().info(f'Trying pose {i}, the quaternion is {placing_pose}.')
            
            x=0.5178879634425592
            y=-0.4814145813368711
            z=0.5179544730306757
            w=-0.48140961983022185
            desired_quaternion = [w, x, y, z]
            trying_quaternion = [placing_pose.orientation.w, placing_pose.orientation.x, placing_pose.orientation.y, placing_pose.orientation.z]
            
            similarity = quaternion_difference(desired_quaternion, trying_quaternion)
            
            self.get_logger().warning(f'The similarity is {similarity}.')
            # 发送放置动作
            self.send_pose_goal(placing_pose, velocity_scaling=0.05)
            self.get_logger().info(f'Trying pose {i}.')
            if self.action_succeeded:
                break

        # 微调试管的放置位置
        # 确认试管当前的姿态是y轴朝上的
        ## TODO
        # 读取当前试管的位置
        while True:
            current_tube_pose = self.pose_array.poses[tube_index]
            offset_x = target_placing_pose.position.x - current_tube_pose.position.x
            offset_y = target_placing_pose.position.y - current_tube_pose.position.y
            offset_z = target_placing_pose.position.z - current_tube_pose.position.z
            # TODO：除了检查位置，还需要检查姿态是否正确，有时会不
            
            if (math.sqrt(offset_x**2 + offset_y**2 + offset_z**2)) > 0.001:
                self.get_logger().info('Tube is not placed correctly. Adjusting...')
                print(self.current_robot_pose)
                # :BUG: 这里的机器人位姿似乎没有正常更新，还是用的旧的位姿，sleep 1s 试之后正常了
                self.get_current_robot_state()
                time.sleep(1)
                robot_pose = self.current_robot_pose
                print(self.current_robot_pose)
                robot_pose_in_isaac_world = self.transform_pose(robot_pose, 'world', 'isaac_world')
                robot_pose_in_isaac_world.position.x += offset_x
                robot_pose_in_isaac_world.position.y += offset_y
                robot_pose_in_isaac_world.position.z += offset_z
                
                transformed_placing_pose = self.transform_pose(robot_pose_in_isaac_world, 'isaac_world', 'world')
                self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.005)
            else:
                self.get_logger().info('Tube placed correctly.')
                break

        transformed_placing_pose.position.z -= 0.1  # 将放置位置下移10cm
        self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.003)
        self.send_gripper_goal(0.5)  # 打开夹爪
        self.get_logger().info('Tube placed.')
        self.rack_occupancy[self.in_used_hole_index] = 1  # 更新rack上的占用情况
        
        # 返回到初始位置
        transformed_placing_pose.position.z += 0.1  # 将放置位置下移10cm
        self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.003)
        
        return transformed_placing_pose

    def send_pose_goal(self, pose, velocity_scaling=0.01):
        goal_msg = MoveToPoseAction.Goal()
        goal_msg.px = pose.position.x
        goal_msg.py = pose.position.y
        goal_msg.pz = pose.position.z
        goal_msg.qw = pose.orientation.w
        goal_msg.qx = pose.orientation.x
        goal_msg.qy = pose.orientation.y
        goal_msg.qz = pose.orientation.z
        goal_msg.velocity_scaling = velocity_scaling
        
        self.robot_ready = False  # 设置机器人状态为不准备好
        self.action_succeeded = False  # 设置动作执行状态为失败
        self._pose_client.wait_for_server()
        self._send_goal_future = self._pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda future: self.generic_goal_response_callback(future, action_type="robot"))

        while not self.robot_ready:
            self.get_logger().debug('Waiting for robot to reach the given pose...')
            time.sleep(0.1)

    def send_gripper_goal(self, target_position):
        goal_msg = MoveGripperAction.Goal()
        goal_msg.target_position = target_position

        self.gripper_ready = False  # 设置夹爪状态为不准备好
        self.action_succeeded = False  # 设置动作执行状态为失败
        self._gripper_client.wait_for_server()
        self._send_gripper_goal_future = self._gripper_client.send_goal_async(goal_msg)
        self._send_gripper_goal_future.add_done_callback(lambda future: self.generic_goal_response_callback(future, action_type="gripper"))

        while not self.gripper_ready:
            self.get_logger().debug('Waiting for gripper to reach the target position...')
            time.sleep(0.1)

    def send_joint_position_goal(self, joint_positions, velocity_scaling=0.01):
        goal_msg = MoveToJointPosition.Goal()
        goal_msg.joint_positions = joint_positions  # List of joint positions
        goal_msg.velocity_scaling = velocity_scaling
        
        self.get_logger().info(f'Sending joint position goal: {joint_positions} with velocity scaling: {velocity_scaling}')
        self.robot_ready = False
        self._joint_position_client.wait_for_server()
        self._send_joint_position_goal_future = self._joint_position_client.send_goal_async(goal_msg)
        self._send_joint_position_goal_future.add_done_callback(lambda future: self.generic_goal_response_callback(future, action_type="robot"))

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
            self.action_succeeded = True
            if action_type == "gripper":
                self.gripper_ready = True  # 设置夹爪状态为准备好
                self.get_logger().info('Gripper ready.')
            elif action_type == "robot":
                self.robot_ready = True  # 设置机器人状态为准备好
                self.get_logger().info('Robot ready.')
            time.sleep(2)
        else:
            self.get_logger().info('Action failed!')
            self.action_succeeded = False
            if action_type == "gripper":
                self.gripper_ready = True  # 设置夹爪状态为准备好
                self.get_logger().info('Gripper ready.')
            elif action_type == "robot":
                self.robot_ready = True  # 设置机器人状态为准备好
                self.get_logger().info('Robot ready.')
    
    ########################################
    ########## 以下是一些辅助函数 ############
    ########################################
    # 用于设置姿态数组
    def set_pose_array(self, pose_array):
        self.pose_array = pose_array
        self.get_logger().debug('Pose array updated.')
        
        return None
    
    # 获取当前机器人的位姿
    def get_current_robot_state(self):
        # 通过异步方式调用 service 获取当前的机器人状态
        request = PrintPose.Request()
        future = self._state_client.call_async(request)
        self.get_logger().info('Calling print_current_pose service...')

        # 设定回调，当服务完成时将调用此回调函数
        future.add_done_callback(self.handle_robot_state_response)

        # 等待服务响应成功，超时时间可以根据需求调整
        timeout = 5  # 设定超时时间5秒
        start_time = time.time()
        while self.current_joint_angles is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for robot state response...")
            time.sleep(0.1)
        
        return None
    
    def check_rack_avalaibility(self):
        # 读取self.rack_occupancy，找到第一个不为1的位置，返回该位置的坐标
        for i in range(6):
            for j in range(15):
                if self.rack_occupancy[i, j] == 0:
                    break
            if self.rack_occupancy[i, j] == 0:
                break
            
        hole_index = (i, j)
        hole_x = 0.60 - 0.016 * i # 0.60是第一个孔的x坐标，0.016是孔之间的间隔
        hole_y = 0.401 - 0.016 * j # 0.40是第一个孔的y坐标，0.016是孔之间的间隔
        hole_z = 0.065
        
        self.in_used_hole_index = hole_index
        
        return hole_x, hole_y, hole_z
    
    def handle_robot_state_response(self, future):
        # 这个函数会在服务调用完成时被触发
        self.get_logger().info('Service call completed.')

        try:
            # 获取服务响应数据
            response = future.result()
            self.current_robot_pose = response.pose  # 保存当前机器人的位姿
            self.current_joint_angles = response.joint_angles  # 保存当前机器人的关节角度
            self.get_logger().info(f"Current robot pose: {self.current_robot_pose}")
            self.get_logger().info(f"Current joint angles: {self.current_joint_angles}")
            
            return None

        except Exception as e:
            self.get_logger().error(f'Failed to call print_current_pose service: {e}')
            
            return None
    
    # 姿态变换函数
    def transform_pose(self, pose, source_frame, target_frame):
        while True:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                break
            except TransformException as ex:
                self.get_logger().debug(f'Could not transform {source_frame} to {target_frame}: {ex}')
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, t)
        return transformed_pose
    
    def calculate_grasping_poses(self, pose):
        '''
        根据输入的管子姿态 (Pose) 计算抓取和预抓取的姿态。该函数会根据管子的当前方向和位置，计算出两个姿态：
        - 预抓取姿态：机器人在执行抓取动作之前的准备位置。
        - 抓取姿态：机器人执行抓取动作时的具体位置。

        args: pose (geometry_msgs.msg.Pose): 待抓取tube的姿态。
        
        return: pre_grasp_pose, grasp_pose
        '''
        # 从管子的姿态中提取位置和四元数, 并使用 quat2mat 将其转换为 3x3 旋转矩阵，以方便后续的几何计算。
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        rotation_matrix = quat2mat(q)
        
        ###################################
        # 以下是计算预抓取和抓取姿态的具体步骤
        ###################################
        
        # 提取tube的当前 y 轴方向 （主轴方向），设定世界坐标系中的 z 轴方向
        tube_y_axis = rotation_matrix[:, 1]
        world_z_direction = np.array([0, 0, 1])
        
        # 计算 tube y 轴与世界 z 轴的叉乘，得到一个垂直于 tube y 轴和世界 z 轴的水平向量，随后正则化
        vector_horiz = np.cross(tube_y_axis, world_z_direction)
        vector_horiz = vector_horiz / np.linalg.norm(vector_horiz)
        
        # 计算 tube y 轴与水平向量的叉乘，得到一个垂直于 tube y 轴和水平向量的向量 (在tube倾斜或者平躺时，这个方向是近似向上的)
        up_from_tube_vector = np.cross(vector_horiz, tube_y_axis)
        up_from_tube_vector = up_from_tube_vector / np.linalg.norm(up_from_tube_vector)
        
        # 对上一个向量取反，就是抓取时的接近向量，随后将其与水平向量和 tube y 轴组合成一个新的旋转矩阵，作为机器人末端位姿
        approach_vector = -up_from_tube_vector
        approach_rotation_matrix = np.column_stack((vector_horiz, -tube_y_axis, approach_vector))
        approach_quaternion = mat2quat(approach_rotation_matrix)
        
        # 计算抓取和预抓取的位置, 使用tube的原点位置加上给定方向上的偏移量
        # tube 的原点位置位于玻璃部分最底端
        tube_origin = np.array([pose.position.x, pose.position.y, pose.position.z])
        pre_grasp_point = tube_origin + up_from_tube_vector * 0.1 + tube_y_axis * 0.07 # 对于 75mm 长的管子，7 cm 可以抓取到盖子
        grasp_point = tube_origin + up_from_tube_vector * 0.0 + tube_y_axis * 0.07
        
        pre_grasp_pose = construct_pose(pre_grasp_point, approach_quaternion)
        grasp_pose = construct_pose(grasp_point, approach_quaternion)

        return pre_grasp_pose, grasp_pose
    
    ########################################
    ########## 以上是一些辅助函数 ############
    ########################################

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

# ros2 action send_goal /execute_grasp_action ur5_robot_gripper/action/ExecuteGrasp "{tube_index: 0}"