#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from ur5_robot_gripper.srv import PoseTransform 

class PoseTransformService(Node):
    def __init__(self):
        super().__init__('pose_transform_service')
        self.srv = self.create_service(PoseTransform, 'transform_pose', self.handle_transform_pose)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("Pose Transform Service is ready.")

    def handle_transform_pose(self, request, response):
        try:
            # 获取从 source_frame 到 target_frame 的变换
            transform = self.tf_buffer.lookup_transform(
                request.target_frame,
                request.pose.header.frame_id,  # source_frame
                rclpy.time.Time(),  # 获取最新的变换
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 应用变换到姿态
            transformed_pose = tf2_geometry_msgs.do_transform_pose(request.pose, transform)
            response.transformed_pose = transformed_pose
            self.get_logger().info(f"Pose transformed to frame {request.target_frame}.")
            return response
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"Transform failed: {ex}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
