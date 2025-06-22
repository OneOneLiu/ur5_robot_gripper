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

class BaseToVirtualLinkPublisher(Node):

    def __init__(self):
        super().__init__('base_to_virtual_link_publisher')

        # 初始化 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建定时器，定时发布 TF
        self.timer = self.create_timer(0.01, self.publish_transforms)

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()
        transforms = []
        
        '''
        在当前设置下，两个坐标系重合，所以不再发布两者关系，直接使用base_link即可
        这个关系是由在isaac sim中载入的usd模型的位姿，以及urdf中定义的base_link
        的位姿决定，urdf一般不会修改，如果在usd中修改了模型的位姿，需要在这里同步
        修改。
        '''
        
        # # -------------------------------
        # # TF1: world -> isaac_world
        # # -------------------------------
        # t1 = TransformStamped()
        # t1.header.stamp = now
        # t1.header.frame_id = 'isaac_world'  # 这里的 base_link 是 URDF 中定义的 base_link
        # t1.child_frame_id = 'base_link'

        # t1.transform.translation.x = 0.0
        # t1.transform.translation.y = 0.0
        # t1.transform.translation.z = 0.0

        # q1 = transforms3d.euler.euler2quat(0, 0, math.radians(0)) 
        # t1.transform.rotation.x = q1[1]
        # t1.transform.rotation.y = q1[2]
        # t1.transform.rotation.z = q1[3]
        # t1.transform.rotation.w = q1[0]
        # transforms.append(t1)

        # -------------------------------
        # TF2: tool0 -> camera_link
        # -------------------------------
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'tool0'
        t2.child_frame_id = 'camera_link'

        t2.transform.translation.x = 0.0
        t2.transform.translation.y = -0.1
        t2.transform.translation.z = 0.0

        q2 = transforms3d.euler.euler2quat(0, 0, math.radians(90)) 
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]
        t2.transform.rotation.w = q2[0]
        transforms.append(t2)

        # ✅ 一次性广播所有 TF
        self.tf_broadcaster.sendTransform(transforms)

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
