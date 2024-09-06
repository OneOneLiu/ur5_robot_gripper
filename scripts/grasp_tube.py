#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ur5_robot_gripper.srv import MoveToPosition
from ur5_robot_gripper.srv import PoseTransform
import tf2_ros
import tf2_geometry_msgs
import time
import threading

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.get_logger().info('PoseSubscriber node initialized.')

        # Create a client for the MoveToPosition service
        self.move_to_position_client = self.create_client(MoveToPosition, 'move_to_position')
        self.get_logger().info('MoveToPosition service client created.')
        # while not self.move_to_position_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for move_to_position service to become available...')
        # self.get_logger().info('MoveToPosition service is available.')
        
        self.tf_client = self.create_client(PoseTransform, 'transform_pose')
        self.get_logger().info('PoseTransform service client created.')
        # while not self.tf_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for PoseTransform service to become available...')
        # self.get_logger().info('PoseTransform service is available.')

        # 订阅 /tube75_poses 话题
        self.subscription = self.create_subscription(
            PoseArray,
            '/tube75_poses',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /tube75_poses topic.')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg):
        self.get_logger().info('Received PoseArray message.')

        # 如果姿态数组小于14，记录警告信息
        if len(msg.poses) < 14:
            self.get_logger().warn('Received PoseArray has less than 14 poses.')
            return 0
        
        self.pose_msg = msg
        threading.Thread(target=self.move_to_tube_poses).start()
    
    def move_to_tube_poses(self):
        # 循环遍历从第1到第14个姿态
        for i in range(2):
            target_pose = Pose()
            target_pose.position.x = -0.2
            target_pose.position.y = 0.5
            target_pose.position.z = 0.2
            target_pose.orientation.w = 1.0
            self.get_logger().warn(f'target_pose_type: {type(target_pose)}')
            # 打印目标姿态的位置和方向
            self.get_logger().info(f'Moving to pose {i + 1}...')
            self.get_logger().info(f'Pose {i + 1} Position: x={target_pose.position.x:.3f}, '
                                   f'y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
            self.get_logger().info(f'Pose {i + 1} Orientation: x={target_pose.orientation.x:.3f}, '
                                   f'y={target_pose.orientation.y:.3f}, z={target_pose.orientation.z:.3f}, '
                                   f'w={target_pose.orientation.w:.3f}')

            # self.get_logger().warn(f'source frame: {self.pose_msg.header.frame_id}')
            # transformed_pose = self.transform_pose(target_pose, target_frame='base_link', source_frame=self.pose_msg.header.frame_id)
            transformed_pose = self.transform_pose(target_pose, target_frame='base_link', source_frame='isaac_world')
            
            self.get_logger().info(f'Transformed Position: x={transformed_pose.position.x:.3f}, '
                                   f'y={transformed_pose.position.y:.3f}, z={transformed_pose.position.z:.3f}')
            self.get_logger().info(f'Transformed Orientation: x={transformed_pose.orientation.x:.3f}, '
                                   f'y={transformed_pose.orientation.y:.3f}, z={transformed_pose.orientation.z:.3f}, '
                                   f'w={transformed_pose.orientation.w:.3f}')
            # # 使用服务请求来移动到指定位置
            self.call_move_to_position_service(
                transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z
            )
    
    # TODO:这里改成action
    def call_move_to_position_service(self, x, y, z):
        self.get_logger().info(f'Calling MoveToPosition service with x={x}, y={y}, z={z}.')
        
        # 创建服务请求
        request = MoveToPosition.Request()
        request.px = x
        request.py = y
        request.pz = z + 0.1  # 为了避免碰撞，将 z 坐标提高 0.2 米

        # 发送请求并等待结果
        future = self.move_to_position_client.call_async(request)
        self.get_logger().info('MoveToPosition service request sent, waiting for response...')

        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is not None:
            self.get_logger().info('MoveToPosition service call succeeded.')
        else:
            self.get_logger().error('MoveToPosition service call failed.')

    def call_pose_transform_service(self, pose_stamped, target_frame, source_frame):
        self.get_logger().info(f'Calling PoseTransform service')
        
        # 创建服务请求
        request = PoseTransform.Request()
        request.pose = pose_stamped
        request.target_frame = target_frame
        request.source_frame = source_frame
        # 发送请求并等待结果
        future = self.tf_client.call_async(request)
        self.get_logger().info(f'PoseTransform service request sent with pose: {request.pose}, waiting for response...')
        # 使用 spin_until_future_complete 确保等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        try:
            self.get_logger().info("I am here3.")
            response = future.result()
            # 添加检查，确保 response 不是 None
            if response is not None:
                self.get_logger().info(f"Transformed Pose: {response.transformed_pose}")
                return response.transformed_pose
            else:
                self.get_logger().error("Received None response from PoseTransform service.")
                return None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        return response.transformed_pose
    
    def transform_pose(self, pose, target_frame, source_frame):
        try:
            if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0)):
                self.get_logger().info(f'Transform from {source_frame} to {target_frame} is available.')
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,  # source_frame
                    rclpy.time.Time(),  # 获取最新的变换
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )
                return tf2_geometry_msgs.do_transform_pose(pose, transform)
            else:
                self.get_logger().error(f'Cannot transform from {source_frame} to {target_frame}. Transform not available.')
                return None
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")

        return None

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('main').info('Initializing ROS...')

    # 创建主节点
    node = PoseSubscriber()
    rclpy.logging.get_logger('main').info('Main node created.')

    # 创建并启动多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    rclpy.logging.get_logger('main').info('Starting to spin with multi-threaded executor.')

    # 在单独的线程中启动执行器
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    node.move_to_tube_poses()
    # 等待执行器线程结束
    executor_thread.join()

    rclpy.logging.get_logger('main').info('Shutting down ROS...')
    
    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
