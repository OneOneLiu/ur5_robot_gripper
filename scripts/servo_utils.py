#!/usr/bin/env python3

'''
作者：Daohui Liu
邮箱：daohui.liu@mail.utoronto.ca
@2025-08-03

本文件用于实现一些moveit的servo的工具函数，包括：
- 设定伺服控制的模式：JointJog, Twist, Pose
- 通过伺服控制姿态的相对调整，如沿某个方向前进，或者绕某个轴旋转，这里的坐标系可以使用当前tf系统中的任意坐标系
- 通过伺服控制完成指定轨迹跟踪，如画圆
- 通过伺服控制完整指定姿态跟踪，如对齐某个轴（欠约束）
- 通过伺服控制完成指定轨迹及姿态跟踪，如画圆并保持某个姿态始终朝向某个点，用于收集固定位置的物体数据
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType
from control_msgs.msg import JointJog
import numpy as np
import time
from enum import Enum
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class ServoMode(Enum):
    """伺服控制模式枚举"""
    JOINT_JOG = 1
    TWIST = 2
    POSE = 3

class TrajectoryType(Enum):
    """轨迹类型枚举"""
    CIRCLE = 1
    ALIGN_AXIS = 2
    CIRCLE_WITH_AXIS_ALIGNMENT = 3 # 画圆的同时让末端z轴始终朝向指定点

class ServoUtils(Node):
    def __init__(self):
        super().__init__('servo_utils')
        
        # 伺服控制话题
        self.twist_topic = '/servo_node/delta_twist_cmds'
        self.joint_topic = '/servo_node/delta_joint_cmds'
        self.pose_topic = '/servo_node/delta_pose_cmds'
        
        # 模式切换服务
        self.switch_service = '/servo_node/switch_command_type'
        
        # 发布器
        self.twist_publisher_ = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.joint_publisher_ = self.create_publisher(JointJog, self.joint_topic, 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, self.pose_topic, 10)
        
        # 订阅器 - 订阅仿真/真实机器人控制节点所发布的当前机器人姿态
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped, 
            '/current_pose', 
            self.pose_callback, 
            10
        )
        
        # 当前姿态相关变量
        self.current_pose = None
        self.current_position = None
        self.current_orientation = None
        self.current_x_axis = None
        self.current_y_axis = None
        self.current_z_axis = None
        
        # 服务客户端
        self.switch_client = self.create_client(ServoCommandType, self.switch_service)
        
        # 当前模式，默认是TWIST模式
        self.current_mode = ServoMode.TWIST
        self.switch_mode(self.current_mode)
        
        # 定时器和状态
        self.start_time = time.time()
        self.timer_period = 0.008  # 125Hz
        self.timer = None
        self.is_active = False
        self.trajectory_type = None
        
        # 轨迹参数
        self.radius = 0.20
        self.circle_angular_speed = 2.0  # 画圆的角速度
        self.frame_id = 'base_link'
        
        # 圆的坐标系
        self.u = None  # 径向向量
        self.v = None  # 切向向量
        self.w = None  # 轴向向量
        
        # 轴对齐参数
        self.align_end_effector_axis = None
        self.align_target_axis = None
        self.align_angular_speed = 0.1  # 轴对齐的角速度
        
        # 朝向跟踪参数
        self.target_point = None
        
        self.get_logger().info("ServoUtils initialized")

        # 给订阅器一点时间接收初始数据
        time.sleep(0.5)

        # 订阅tf，从而获取相机到base_link的变换关系
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 测试画圆和轴对齐
        self.circle_with_axis_alignment(
            radius=0.1, 
            circle_angular_speed=0.2, 
            axis_direction=[0, 0, -1], 
            axis_position=[0.1, 0, 0], 
            end_effector_axis='z',
            target_axis=[0, 0, -1],
            target_point=[0.5, -0.12, 0.1],
            align_angular_speed=0.3,
            frame_id='base_link'
        )

        # # 测试画圆
        # self.draw_circle_trajectory(
        #     radius=0.3, 
        #     circle_angular_speed=0.6, 
        #     axis_direction=[0, 0, 1], 
        #     axis_position=[0.1, 0, 0],
        #     frame_id='base_link'
        # )

        # # 测试轴对齐
        # self.align_axis(
        #     end_effector_axis='z',
        #     target_axis=[0, 0, -1],
        #     angular_speed=0.1,
        #     publish_twist=True
        # )

    def switch_mode(self, mode: ServoMode) -> bool:
        """切换伺服控制模式"""
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Switch service not available")
            return False
            
        request = ServoCommandType.Request()
        
        if mode == ServoMode.JOINT_JOG:
            request.command_type = ServoCommandType.Request.JOINT_JOG
        elif mode == ServoMode.TWIST:
            request.command_type = ServoCommandType.Request.TWIST
        elif mode == ServoMode.POSE:
            request.command_type = ServoCommandType.Request.POSE
        else:
            self.get_logger().error(f"Unknown mode: {mode}")
            return False
            
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None and future.result().success:
            self.current_mode = mode
            self.get_logger().info(f"Switched to mode: {mode.name}")
            return True
        else:
            self.get_logger().warn(f"Failed to switch to mode: {mode.name}")
            return False

    def ensure_twist_mode(self):
        """确保使用TWIST模式"""
        if self.current_mode != ServoMode.TWIST:
            return self.switch_mode(ServoMode.TWIST)
        return True

    def create_twist_message(self, linear_vel, angular_vel):
        """创建TwistStamped消息"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(linear_vel[0])
        msg.twist.linear.y = float(linear_vel[1])
        msg.twist.linear.z = float(linear_vel[2])
        msg.twist.angular.x = float(angular_vel[0])
        msg.twist.angular.y = float(angular_vel[1])
        msg.twist.angular.z = float(angular_vel[2])
        return msg

    def draw_circle_trajectory(self, radius=0.1, circle_angular_speed=1.0, axis_direction=[0, 1, 0], 
                             axis_position=[-0.1, 0, 0], frame_id='finger_center', publish_twist=True):
        """
        开始画圆轨迹
        
        Args:
            radius: 圆的半径 (m), 默认0.1m
            angular_speed: 角速度 (rad/s), 默认1.0rad/s
            axis_direction: 旋转轴方向 [rx, ry, rz], 默认[0, 0, 1], 即z轴
            axis_position: 旋转轴位置 [x, y, z]（参考点）, 默认[0.1, 0, 0], 即轴位置在相对末端执行器位置x=0.1m处
            frame_id: 坐标系ID, 默认'base_link'

        注意：
        - 本函数仅能参考当前的位置进行局部坐标系下的画圆，使用时建议先通过motion planning来运动到想要的初始位置，然后使用本函数进行画圆
        - 画圆时，速度和半径都不要设置过大，否则容易导致执行不到位而偏离轨迹，带来危险情形。不过如果moveit的planning scene中设置了正确的碰撞场景，倒是不会出现碰撞，最多在靠近障碍物时减速乃至停止。

        """
        if not self.ensure_twist_mode():
            return False
            
        self.radius = radius
        self.circle_angular_speed = circle_angular_speed
        self.frame_id = frame_id
            
        if not self.setup_circle_coordinate_system(axis_direction, axis_position):
            return False

        self.start_time = time.time()
        self.is_active = True

        if self.timer is None and publish_twist:
            self.get_logger().info(f"Started circle trajectory: radius={radius}m, speed={circle_angular_speed}rad/s")
            self.trajectory_type = TrajectoryType.CIRCLE
            self.timer = self.create_timer(self.timer_period, self.trajectory_timer_callback)
        
        self.get_logger().info(f"Started circle trajectory: radius={radius}m, speed={circle_angular_speed}rad/s")
    
    def setup_circle_coordinate_system(self, axis_direction, axis_position):
        """
        计算并设置画圆轨迹的坐标系
        
        Args:
            axis_direction: 旋转轴方向 [rx, ry, rz]
            axis_position: 旋转轴位置 [x, y, z]（相对于末端执行器的位置）
        """
        # 归一化轴方向
        self.w = np.array(axis_direction) / np.linalg.norm(axis_direction)
        axis_pos = np.array(axis_position)
        
        # 检查轴位置向量是否为零
        if np.linalg.norm(axis_pos) < 1e-6:
            self.get_logger().error("Axis position cannot be at origin [0,0,0].")
            return False
        
        # 计算径向向量（从轴位置到末端的向量，垂直于轴）
        projection_length = np.dot(axis_pos, self.w)
        to_end_effector = -axis_pos - projection_length * self.w
        
        # 如果径向向量接近零，说明末端执行器在旋转轴上
        # 这种情况下，我们需要选择一个默认的径向方向
        if np.linalg.norm(to_end_effector) < 1e-6:
            self.get_logger().warn("End effector is on the rotation axis. Using default radial direction.")
            # 选择一个垂直于旋转轴的默认方向
            if abs(self.w[2]) < 0.9:  # 如果轴不是垂直的，用z轴作为参考
                default_direction = np.array([0, 0, 1])
            else:  # 如果轴接近垂直，用x轴作为参考
                default_direction = np.array([1, 0, 0])
            
            # 计算垂直于轴的向量
            to_end_effector = np.cross(self.w, default_direction)
            if np.linalg.norm(to_end_effector) < 1e-6:  # 如果叉积接近零，换一个方向
                to_end_effector = np.cross(self.w, np.array([0, 1, 0]))
        
        # 归一化径向向量作为u轴
        self.u = to_end_effector / np.linalg.norm(to_end_effector)
        
        # 计算切向向量：v = w × u
        self.v = np.cross(self.w, self.u)
        self.v = self.v / np.linalg.norm(self.v)
        
        # 重新计算u轴确保正交：u = v × w
        self.u = np.cross(self.v, self.w)
        self.u = self.u / np.linalg.norm(self.u)
        
        self.get_logger().info(f"Circle coordinate system set up")
        self.get_logger().info(f"  U axis (radial): {self.u}")
        self.get_logger().info(f"  V axis (tangential): {self.v}")
        self.get_logger().info(f"  W axis (axial): {self.w}")
        return True

    def align_axis(self, end_effector_axis='z', target_axis=[0, 0, -1], angular_speed=0.1, publish_twist=True):
        """
        对齐末端的指定轴到指定方向（欠约束控制）
        
        Args:
            end_effector_axis: 末端执行器轴，可选'x', 'y', 'z'
            target_axis: 目标轴方向 [x, y, z]
            angular_speed: 角速度大小 (rad/s)
        
        注意：
        - 当前的这个方法只是沿着最小化误差的方向进行旋转，因此并不智能，如果旋转中途因为姿态过于别扭而无法到达，它不会自动停止或者调整，直到有碰撞出现，moveit迫使它停止。
            打个比方，如果我们要控制末端执行器的z轴从跟世界的z轴重合旋转到世界z轴的反方向，那么这个方法会直接沿着最近的方向去转，中间机械臂会扭曲产生自干涉导致无法转过去，
            而实际上从不管是正z还是-z都是可达的。也有可能是因为我们只给了角速度，没有给线速度，导致机械臂在旋转过程始终保持在该点，相当于加了个三轴的约束，这样从正z转过去-z就变得不可达了。
        """
        if not self.ensure_twist_mode():
            return False
        
        if self.current_pose is None:
            self.get_logger().warn("No current pose available. Will start alignment when pose data is received.")
            # 不阻塞，继续启动定时器，让定时器回调等待姿态数据
        
        # 获取当前轴向量（如果可用）
        axis_map = {'x': self.current_x_axis, 'y': self.current_y_axis, 'z': self.current_z_axis}
        if end_effector_axis not in axis_map:
            self.get_logger().error(f"Invalid end effector axis: {end_effector_axis}")
            return False
        
        # 保存对齐参数
        self.align_end_effector_axis = end_effector_axis
        self.align_target_axis = np.array(target_axis) / np.linalg.norm(target_axis)
        self.align_angular_speed = angular_speed
        
        self.is_active = True
        if self.timer is None and publish_twist:
            self.trajectory_type = TrajectoryType.ALIGN_AXIS
            self.timer = self.create_timer(self.timer_period, self.trajectory_timer_callback)
        
        self.get_logger().info(f"Started axis alignment: {end_effector_axis}-axis to {target_axis}")
    
    def circle_with_axis_alignment(self, radius=0.1, circle_angular_speed=1.0, axis_direction=[0, 1, 0], axis_position=[-0.1, 0, 0], end_effector_axis='z', 
            target_axis=[0, 0, -1], target_point=None, align_angular_speed=0.1, frame_id='finger_center'):
        """
        画圆并保持朝向指定点
        """
        self.trajectory_type = TrajectoryType.CIRCLE_WITH_AXIS_ALIGNMENT
        
        self.draw_circle_trajectory(
            radius=radius, 
            circle_angular_speed=circle_angular_speed, 
            axis_direction=axis_direction, 
            axis_position=axis_position, 
            frame_id=frame_id,
            publish_twist=False
        )
        if target_point is not None:
            self.target_point = target_point
            target_axis = self._calculate_target_axis_from_target_point(target_point)
        
        self.align_axis(
            end_effector_axis=end_effector_axis, 
            target_axis=target_axis, 
            angular_speed=align_angular_speed,
            publish_twist=False
        )

        self.is_active = True
        if self.timer is None:
            self.timer = self.create_timer(self.timer_period, self.trajectory_timer_callback)


    def trajectory_timer_callback(self):
        """统一的轨迹定时器回调"""
        if not self.is_active:
            return
            
        if self.trajectory_type == TrajectoryType.CIRCLE:
            self._circle_callback()
        elif self.trajectory_type == TrajectoryType.ALIGN_AXIS:
            self._align_axis_callback()
        elif self.trajectory_type == TrajectoryType.CIRCLE_WITH_AXIS_ALIGNMENT:
            self._circle_with_axis_alignment_callback()

    def _circle_callback(self):
        """画圆轨迹的具体实现"""
        t = time.time() - self.start_time
        linear = self._calculate_circle_velocity(t)
        angular = np.array([0.0, 0.0, 0.0])
        
        self.linear_velocity = linear

        # 如果轨迹类型是画圆，则发布画圆的线速度，否则不发布，仅计算
        if self.trajectory_type == TrajectoryType.CIRCLE:
            msg = self.create_twist_message(linear, angular)
            self.twist_publisher_.publish(msg)

    def _align_axis_callback(self):
        """轴对齐的具体实现，修正为物理正确的向量对齐方式"""
        if self.current_pose is None:
            return

        angular = self._calculate_alignment_angular_velocity()
        linear = np.array([0.0, 0.0, 0.0])

        self.angular_velocity = angular

        # 如果轨迹类型是轴对齐，则发布轴对齐的角速度，否则不发布，仅计算
        if self.trajectory_type == TrajectoryType.ALIGN_AXIS:
            msg = self.create_twist_message(linear, angular)
            self.twist_publisher_.publish(msg)
    
    def _circle_with_axis_alignment_callback(self):
        self.get_logger().info(f"Started circle with axis alignment")
        self._circle_callback()

        # 获取相机到base_link的变换关系
        camera_pose = self.get_camera_pose()
        self.get_logger().info(f"Camera pose: {camera_pose}")

        # 实时更新要朝向的目标轴向量
        self.align_target_axis = self._calculate_target_axis_from_target_point(self.target_point, camera_pose[0])
        self._align_axis_callback()
        self.get_logger().info(f"Started circle with axis alignment: linear velocity: {self.linear_velocity}, angular velocity: {self.angular_velocity}")

        msg = self.create_twist_message(self.linear_velocity, self.angular_velocity)
        self.twist_publisher_.publish(msg)

    def _calculate_circle_velocity(self, t):
        """计算画圆的线速度"""
        theta = self.circle_angular_speed * t
        return self.radius * self.circle_angular_speed * (-np.sin(theta) * self.u + np.cos(theta) * self.v)

    def _calculate_alignment_angular_velocity(self):
        """计算轴对齐的角速度"""
        if self.current_pose is None:
            return np.array([0.0, 0.0, 0.0])
            
        # 获取当前轴向量
        axis_map = {
            'x': self.current_x_axis,
            'y': self.current_y_axis,
            'z': self.current_z_axis
        }
        current_axis = axis_map[self.align_end_effector_axis]

        # 归一化向量
        current_axis = current_axis / np.linalg.norm(current_axis)
        target_axis = self.align_target_axis / np.linalg.norm(self.align_target_axis)

        # 计算旋转轴（叉乘）和旋转角（点乘）
        rot_axis = np.cross(current_axis, target_axis)
        rot_norm = np.linalg.norm(rot_axis)

        if rot_norm < 1e-4:
            # 已对齐或夹角接近 180°（此时旋转轴不唯一）
            return np.array([0.0, 0.0, 0.0])
        else:
            rot_axis = rot_axis / rot_norm
            angle_error = np.arccos(np.clip(np.dot(current_axis, target_axis), -1.0, 1.0))

            if angle_error < 0.01:
                # 对齐完成，停止角速度
                return np.array([0.0, 0.0, 0.0])
            else:
                # 构造角速度向量
                return rot_axis * self.align_angular_speed
    
    def _calculate_target_axis_from_target_point(self, target_point, current_position = None):
        """计算目标轴向量"""
        target_point = np.array(target_point)
        if current_position is None:
            current_position = self.current_position
        
        # 计算目标轴向量
        target_axis = target_point - current_position
        target_axis = target_axis / np.linalg.norm(target_axis)

        return target_axis

    def stop_trajectory(self):
        """停止当前轨迹"""
        self.is_active = False
        if self.timer is not None:
            self.timer.destroy()
            self.timer = None
        
        # 发送零速度命令
        if self.current_mode == ServoMode.TWIST:
            zero_vel = np.array([0.0, 0.0, 0.0])
            msg = self.create_twist_message(zero_vel, zero_vel)
            self.twist_publisher_.publish(msg)
        
        self.get_logger().info("Trajectory stopped")

    def set_joint_velocity(self, joint_velocities):
        """
        设置关节速度（JointJog模式）
        
        Args:
            joint_velocities: 关节速度列表 [rad/s]
        """
        if self.current_mode != ServoMode.JOINT_JOG:
            self.switch_mode(ServoMode.JOINT_JOG)
        
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        msg.velocities = joint_velocities
        
        self.joint_publisher_.publish(msg)

    def set_pose_delta(self, pose_delta):
        """
        设置位姿增量（Pose模式）
        
        Args:
            pose_delta: 位姿增量 [x, y, z, roll, pitch, yaw]
        """
        if self.current_mode != ServoMode.POSE:
            self.switch_mode(ServoMode.POSE)
        
        # 这里需要实现位姿增量控制
        # 需要根据实际需求实现
        self.get_logger().info(f"Setting pose delta: {pose_delta}")

    def pose_callback(self, msg):
        """
        订阅当前机器人姿态的回调函数
        """
        self.current_pose = msg
        self.current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.current_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, 
                                           msg.pose.orientation.z, msg.pose.orientation.w])
        
        # 从四元数中提取当前xyz轴向量
        # 四元数转旋转矩阵，然后提取轴向量
        self.current_x_axis, self.current_y_axis, self.current_z_axis = self.quaternion_to_axes(self.current_orientation)

    def quaternion_to_axes(self, quaternion):
        """
        将四元数转换为xyz轴向量
        
        Args:
            quaternion: [x, y, z, w] 四元数
            
        Returns:
            x_axis, y_axis, z_axis: 三个轴向量
        """
        x, y, z, w = quaternion
        
        # 四元数转旋转矩阵
        # 旋转矩阵的列向量就是xyz轴向量
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
            [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        # 提取轴向量（旋转矩阵的列向量）
        x_axis = rotation_matrix[:, 0]  # 第一列
        y_axis = rotation_matrix[:, 1]  # 第二列  
        z_axis = rotation_matrix[:, 2]  # 第三列
        
        return x_axis, y_axis, z_axis
    
    def get_camera_pose(self, reference_frame='base_link', camera_link='camera_color_frame'):
        try:
            # 取最新的 transform（0 时刻代表最新）
            t: TransformStamped = self.tf_buffer.lookup_transform(
                reference_frame, camera_link, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None

        # 位置（在 reference_frame 下）
        p = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        # 四元数（x,y,z,w）
        q = np.array([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ])
        return p, q

def main(args=None):
    rclpy.init(args=args)
    node = ServoUtils()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_trajectory()
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()