#!/usr/bin/env python3
# 必须得有上面这一句shepang才能在终端直接运行这个python文件作为ros的node
import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class BaseToVirtualLinkPublisher(Node):

    def __init__(self):
        super().__init__('base_to_virtual_link_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to periodically send the transform
        self.timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        t = TransformStamped()

        # Set the transform header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # Parent frame
        t.child_frame_id = 'isaac_world'     # Child frame

        # Set the translation (assuming no translation, only rotation)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set the rotation (around the Z axis)
        theta = math.radians(-90)
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = BaseToVirtualLinkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
