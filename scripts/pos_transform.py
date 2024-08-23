#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import transforms3d.quaternions as quat

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')

        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the previously published PoseStamped message
        self.subscription = self.create_subscription(
            PoseStamped,
            'object_pose',  # Replace this with the actual topic name
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        # Try to get the transform from isaac_world to base_link
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'isaac_world', rclpy.time.Time())

            # Apply the transform to the pose
            transformed_pose = self.apply_transform(msg.pose, trans.transform)

            # Print the transformed pose
            self.get_logger().info(f"Transformed Position: x={transformed_pose.position.x}, y={transformed_pose.position.y}, z={transformed_pose.position.z}")
            self.get_logger().info(f"Transformed Orientation: x={transformed_pose.orientation.x}, y={transformed_pose.orientation.y}, z={transformed_pose.orientation.z}, w={transformed_pose.orientation.w}")

        except Exception as e:
            self.get_logger().warn(f'Failed to transform pose: {str(e)}')

    def apply_transform(self, pose, transform):
        # Apply the transform to the pose
        transformed_pose = PoseStamped().pose

        # Transform the position
        transformed_pose.position.x = transform.translation.x + pose.position.x
        transformed_pose.position.y = transform.translation.y + pose.position.y
        transformed_pose.position.z = transform.translation.z + pose.position.z

        # Transform the orientation (rotation)
        q1 = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        q2 = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        
        # Apply quaternion multiplication
        q_result = quat.qmult(q2, q1)

        transformed_pose.orientation.x = q_result[0]
        transformed_pose.orientation.y = q_result[1]
        transformed_pose.orientation.z = q_result[2]
        transformed_pose.orientation.w = q_result[3]

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)

    # Create the ROS node and start processing callbacks
    node = PoseTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
