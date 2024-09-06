#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from ur5_robot_gripper.srv import MoveToPosition
from ur5_robot_gripper.srv import PoseTransform
import time
import threading

class PoseSubscriber(Node):
    def __init__(self, node):
        super().__init__('pose_subscriber')
        self.get_logger().info('PoseSubscriber node initialized.')

        # Create a client for the MoveToPosition service
        self.move_to_position_client = node.create_client(MoveToPosition, 'move_to_position')
        self.get_logger().info('MoveToPosition service client created.')
        while not self.move_to_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_to_position service to become available...')
        self.get_logger().info('MoveToPosition service is available.')
        
        self.tf_client = self.create_client(PoseTransform, 'transform_pose')
        self.get_logger().info('PoseTransform service client created.')
        while not self.tf_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PoseTransform service to become available...')
        self.get_logger().info('PoseTransform service is available.')

        # 订阅 /tube75_poses 话题
        self.subscription = self.create_subscription(
            PoseArray,
            '/tube75_poses',
            self.pose_callback,
            10
        )
        self.get_logger().info('Subscribed to /tube75_poses topic.')

    def pose_callback(self, msg):
        self.get_logger().info('Received PoseArray message.')

        # 如果姿态数组小于14，记录警告信息
        if len(msg.poses) < 14:
            self.get_logger().warn('Received PoseArray has less than 14 poses.')
            return

        # 循环遍历从第1到第14个姿态
        for i in range(14):
            target_pose = msg.poses[i]
            self.get_logger().warn(f'target_pose_type: {type(target_pose)}')
            # 打印目标姿态的位置和方向
            self.get_logger().info(f'Moving to pose {i + 1}...')
            self.get_logger().info(f'Pose {i + 1} Position: x={target_pose.position.x:.3f}, '
                                   f'y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
            self.get_logger().info(f'Pose {i + 1} Orientation: x={target_pose.orientation.x:.3f}, '
                                   f'y={target_pose.orientation.y:.3f}, z={target_pose.orientation.z:.3f}, '
                                   f'w={target_pose.orientation.w:.3f}')


            transformed_pose = self.call_pose_transform_service(target_pose, target_frame='base_link', source_frame=msg.header.frame_id)

            # 使用服务请求来移动到指定位置
            self.call_move_to_position_service(
                self.transformed_pose.position.y,
                self.transformed_pose.position.x,
                self.transformed_pose.position.z
            )

            # 可以选择在每次移动后进行一个小的延时，以确保运动稳定
            time.sleep(2)  # 根据需要调整时间

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
        self.get_logger().info("I am here1.")
        # 发送请求并等待结果
        future = self.tf_client.call_async(request)
        self.get_logger().info("I am here2.")
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
            self.get_logger().info("I am here4.")
            self.get_logger().info(f"Transformed Pose: {response.transformed_pose}")
            self.get_logger().info("I am here5.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        return response.transformed_pose

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('main').info('Initializing ROS...')

    # 创建主节点
    node = rclpy.create_node('main_node')
    rclpy.logging.get_logger('main').info('Main node created.')

    # 创建 PoseSubscriber 实例
    subscriber_node = PoseSubscriber(node)

    # 创建并启动多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(subscriber_node)

    rclpy.logging.get_logger('main').info('Starting to spin with multi-threaded executor.')

    # 在单独的线程中启动执行器
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    # 等待执行器线程结束
    executor_thread.join()

    rclpy.logging.get_logger('main').info('Shutting down ROS...')
    
    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
