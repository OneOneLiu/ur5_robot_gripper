#!/usr/bin/env python3
# 必须要有上面这一句 shebang 才能直接作为 ROS 2 节点运行
import math
import numpy as np
import transforms3d.euler
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
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss
    return q

def build_transforms(parent_frame_id, child_frame_id, translation, quaternion, timestamp):
    t = TransformStamped()
    t.header.stamp = timestamp
    t.header.frame_id = parent_frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    t.transform.rotation.x = quaternion[1]
    t.transform.rotation.y = quaternion[2]
    t.transform.rotation.z = quaternion[3]
    t.transform.rotation.w = quaternion[0]

    return t

class StaticTFPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_publisher')

        # 初始化 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建定时器，定时发布 TF
        self.timer = self.create_timer(0.01, self.publish_transforms)

    def publish_transforms(self):
        transforms = []
        
        '''
        在当前设置下，两个坐标系重合，所以不再发布两者关系，直接使用base_link即可
        这个关系是由在isaac sim中载入的usd模型的位姿，以及urdf中定义的base_link
        的位姿决定，urdf一般不会修改，如果在usd中修改了模型的位姿，需要在这里同步
        修改。
        '''
        timestamp = self.get_clock().now().to_msg()
        
        # # -------------------------------
        # # TF1: world -> isaac_world
        # # -------------------------------
        # t1 = build_transforms('world', 'base_link', [0.0, 0.0, 0.0], transforms3d.euler.euler2quat(0, 0, math.radians(0)), timestamp)
        # transforms.append(t1)
        
        # -------------------------------
        # TF2: tool0 -> camera_link
        # -------------------------------
        t2 = build_transforms('tool0', 'camera_link', [0.0, -0.1, 0.0], transforms3d.euler.euler2quat(0, 0, math.radians(90)), timestamp)
        transforms.append(t2)

        # ✅ 一次性广播所有 TF
        self.tf_broadcaster.sendTransform(transforms)

def main():
    rclpy.init()
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
