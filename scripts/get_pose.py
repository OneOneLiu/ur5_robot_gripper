#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np

# dc=_dynamic_control.acquire_dynamic_control_interface()

# object=dc.get_rigid_body("/World/tube75/tube75_0_0/tube75/tube75")
# object_pose=dc.get_rigid_body_pose(object)

# print("position:", object_pose.p)
# print("rotation:", object_pose.r)

# # https://forums.developer.nvidia.com/t/get-position-of-primitive/146702

class ObjectPosePublisher(Node):
    def __init__(self, object_path):
        super().__init__('object_pose_publisher')
        self.object_path = object_path

        # Initialize the ROS publisher
        self.publisher_ = self.create_publisher(PoseStamped, 'object_pose', 10)

        # Initialize the dynamic control interface
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        # Timer to periodically publish the pose
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        # Get the rigid body handle
        obj = self.dc.get_rigid_body(self.object_path)

        if obj is None:
            self.get_logger().warn(f"Object {self.object_path} not found!")
            return

        # Get the pose of the object in world coordinates
        pose = self.dc.get_rigid_body_pose(obj)

        # Check if the quaternion is valid (non-zero and normalized)
        rotation = np.array([pose.r.x, pose.r.y, pose.r.z, pose.r.w])
        norm = np.linalg.norm(rotation)
        if norm == 0 or not np.isfinite(norm):
            self.get_logger().warn(f"Invalid quaternion received for {self.object_path}. Setting to default identity rotation.")
            rotation = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            rotation /= norm  # Normalize the quaternion

        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'  # You can change this to the relevant frame_id

        # Set the position
        pose_msg.pose.position.x = pose.p.x
        pose_msg.pose.position.y = pose.p.y
        pose_msg.pose.position.z = pose.p.z

        # Set the orientation
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]

        # Publish the pose
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"Published pose of {self.object_path}.")

def main(args=None):
    rclpy.init(args=args)

    # Define the path to your object in the Isaac Sim scene
    object_path = "/World/tube75/tube75_0_0/tube75/tube75"

    # Create the ROS node and start publishing
    node = ObjectPosePublisher(object_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()