#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from ur5_robot_gripper.action import MoveToPositionAction, MoveToPoseAction, MoveGripperAction, ExecuteGrasp, MoveToJointPosition
from ur5_robot_gripper.srv import PrintPose
from rclpy.action import ActionClient, ActionServer
import time
import numpy as np
import math
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.axangles import axangle2mat
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

def pose_to_matrix(pose):
    # 提取平移
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])

    # 提取四元数并转换为旋转矩阵
    quaternion = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    rotation_matrix = quat2mat(quaternion)

    # 构建4x4的变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    return transform_matrix

def apply_transform(robot_pose, relative_transform):
    # 将机器人的姿态转换为矩阵
    robot_matrix = pose_to_matrix(robot_pose)

    # 计算新的机器人姿态矩阵
    new_robot_matrix = robot_matrix.dot(relative_transform)

    # 提取新的平移和旋转
    new_translation = new_robot_matrix[:3, 3]
    new_rotation_matrix = new_robot_matrix[:3, :3]
    
    # 转换为四元数
    from transforms3d.quaternions import mat2quat
    new_quaternion = mat2quat(new_rotation_matrix)

    # 构建新的 Pose
    new_pose = Pose()
    new_pose.position.x = new_translation[0]
    new_pose.position.y = new_translation[1]
    new_pose.position.z = new_translation[2]
    new_pose.orientation.w = new_quaternion[0]
    new_pose.orientation.x = new_quaternion[1]
    new_pose.orientation.y = new_quaternion[2]
    new_pose.orientation.z = new_quaternion[3]

    return new_pose

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
        self._joint_position_client = ActionClient(self, MoveToJointPosition, 'move_to_joint_position_action')  # Added Action Client for MoveToJointPosition
        
        # 创建一个service client，用于获取当前机器人的位姿
        self._state_client = self.create_client(PrintPose, 'print_current_pose')
        while not self._state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for print_current_pose service to become available...')
        
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
        # 存储当前机器人的位姿和关节角度
        self.current_joint_angles = None
        
        self.action_succeeded = True

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
        
        # re_oriented_pose = self.calculate_placing_poses(transformed_pre_grasp_pose)
        # transformed_re_oriented_pose = self.transform_pose(re_oriented_pose, 'isaac_world', 'world')
        transformed_re_oriented_pose = None

        # 执行抓取动作
        self.execute_action_sequence(transformed_pre_grasp_pose, transformed_grasp_pose, transformed_re_oriented_pose, tube_index)

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
        pre_grasp_point = cylinder_center + result_vector * 0.1 + cylinder_y_axis * 0.05
        grasp_point = cylinder_center + result_vector * 0.0 + cylinder_y_axis * 0.05
        
        pre_grasp_pose = construct_pose(pre_grasp_point, quaternion_opposite)
        grasp_pose = construct_pose(grasp_point, quaternion_opposite)

        return pre_grasp_pose, grasp_pose
    
    def calculate_placing_poses(self, pose = None):
        # 先获取当前机器人的状态
        self.get_current_robot_state()

        # 假设已经获取到了self.current_joint_angles，并使用它来进行计算
        if self.current_joint_angles is not None:
            # Modify the 6th joint angle to rotate it 90 degrees clockwise (i.e., -π/2 radians)
            new_joint_angles = self.current_joint_angles[:]
            new_joint_angles[5] -= math.pi / 4  # Rotate the 6th joint 90 degrees clockwise

            self.get_logger().info(f"Modified joint angles with 6th joint rotated: {new_joint_angles}")

            return new_joint_angles
        else:
            self.get_logger().warning('Current joint angles not available yet!')
            return None


    def transform_pose(self, pose, source_frame, target_frame):
        while True:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                break
            except TransformException as ex:
                self.get_logger().debug(f'Could not transform {source_frame} to {target_frame}: {ex}')
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, t)
        return transformed_pose

    def execute_action_sequence(self, pre_grasp_pose, grasp_pose, re_oriented_pose, tube_index):
        # Consolidate sending goals and handling callbacks into a single function
        self.get_logger().info('Executing action sequence...')
        self.send_pose_goal(pre_grasp_pose, velocity_scaling=0.07)
        self.get_logger().info('Pre-grasp pose reached.')
        self.send_gripper_goal(0.5)  # 关闭夹爪
        self.get_logger().info('Gripper closed.')
        self.send_pose_goal(grasp_pose, velocity_scaling=0.01)
        self.get_logger().info('Grasp pose reached.')
        self.send_gripper_goal(0.7)  # 关闭夹爪
        self.get_logger().info('Tube grasped.')
        self.send_pose_goal(pre_grasp_pose, velocity_scaling=0.05)
        self.get_logger().info('Pre-grasp pose reached.')
        # array('d', [-1.536058644578639, -1.2418357984149566, 1.622218157889908, -1.972800967355929, -1.5932304181778447, 0.019746842630313566])
        # geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.35140546093353975, y=-0.6029201291432347, z=0.3716432330812862), orientation=geometry_msgs.msg.Quaternion(x=0.5178879634425592, y=-0.4814145813368711, z=0.5179544730306757, w=-0.48140961983022185))
        # pre_placing_joint_angles = [-math.pi/2, -1.2418357984149566, math.pi/2, -1.972800967355929, 0.0000, -math.pi/2]
        # self.send_joint_position_goal(pre_placing_joint_angles, velocity_scaling=0.05)
        # self.get_logger().info('Action sequence completed.')
        
        # Add placing action here
        self.placing_tube(tube_index)

    def placing_tube(self, tube_index):
        # 设定放置管子的位置，姿态在后面生成
        placing_pose = Pose()
        placing_pose.position.x = 0.6
        placing_pose.position.y = 0.32
        placing_pose.position.z = 0.08 + 0.05 # 设定位置为孔的第一个位置上方5cm
        
        # 获取当前管子的姿态
        current_tube_pose = self.pose_array.poses[tube_index]
        current_tube_rotation_matrix = quat2mat([current_tube_pose.orientation.w,
                                            current_tube_pose.orientation.x,
                                            current_tube_pose.orientation.y,
                                            current_tube_pose.orientation.z])
        current_y_axis = current_tube_rotation_matrix[:, 1]  # 当前的 y 轴方向
        target_y_axis = np.array([0, 0, 1])  # 目标 y 轴是世界坐标系中的 z 轴
        
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
        
        ## tube y轴与世界z轴对齐，tube的 x, z轴可以绕世界z轴旋转，下面生成一组候选的旋转，机器人执行时挨个去试
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

        # 上面求得的旋转矩阵表示试管应当如何进行旋转，才能使得它立着，后面把它变换成机器人的旋转矩阵
        # 获取当前机器人的位姿
        self.get_current_robot_state()
        robot_pose = self.current_robot_pose
        robot_pose_in_isaac_world = self.transform_pose(robot_pose, 'world', 'isaac_world')

        # 计算tube与机器人之间的变换矩阵 （假设它在后续过程中都保持不变）
        robot_matrix = pose_to_matrix(robot_pose_in_isaac_world)
        tube_matrix = pose_to_matrix(current_tube_pose)
        
        # 计算相对变换矩阵
        relative_transform = np.linalg.inv(robot_matrix).dot(tube_matrix)
        
        # 依次尝试每个候选的旋转四元数
        for i, candidate_quaternion in enumerate(candidate_quaternions):
            self.get_logger().info(f'Trying pose {i}, the quaternion is {candidate_quaternion}.')
            placing_pose.orientation.w = candidate_quaternion[0]
            placing_pose.orientation.x = candidate_quaternion[1]
            placing_pose.orientation.y = candidate_quaternion[2]
            placing_pose.orientation.z = candidate_quaternion[3]
            new_robot_pose = apply_transform(placing_pose, relative_transform) # 这个的平移是对的
            
            transformed_placing_pose = self.transform_pose(new_robot_pose, 'isaac_world', 'world')
            
            # 发送放置动作
            self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.05)
            self.get_logger().info(f'Trying pose {i}.')
            if self.action_succeeded:
                break
        
        # 微调试管的放置位置
        # 确认试管当前的姿态是y轴朝上的
        ## TODO
        # 读取当前试管的位置
        while True:
            current_tube_pose = self.pose_array.poses[tube_index]
            offset_x = placing_pose.position.x - current_tube_pose.position.x
            offset_y = placing_pose.position.y - current_tube_pose.position.y
            offset_z = placing_pose.position.z - current_tube_pose.position.z
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
                self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.05)
            else:
                self.get_logger().info('Tube placed correctly.')
                break
        
        transformed_placing_pose.position.z -= 0.08  # 将放置位置下移5cm
        self.send_pose_goal(transformed_placing_pose, velocity_scaling=0.01)
        self.send_gripper_goal(0.5)  # 打开夹爪
        self.get_logger().info('Tube placed.')
        
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

        except Exception as e:
            self.get_logger().error(f'Failed to call print_current_pose service: {e}')

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