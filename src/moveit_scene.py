#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Planning_scene(object):
    """UR_robot moveit control class"""

    def __init__(self):
        super(Planning_scene, self).__init__()

        ## First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        self.scene = moveit_commander.PlanningSceneInterface()

        self.add_box_objects('table', (0, 0, 0), (1.5, 1.5, 0.02))

    def add_box_objects(self, box_name, pose, box_size):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = pose[0]
        box_pose.pose.position.y = pose[1]
        box_pose.pose.position.z = pose[2]
        self.scene.add_box(box_name, box_pose, size = box_size)

def main():
    rospy.init_node("moveit_scene_setup")
    scene = Planning_scene()

    rospy.spin()

if __name__ == "__main__":
    main()