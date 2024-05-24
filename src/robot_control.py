#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pynput.keyboard import Key, Listener

class UR_robot(object):
    """UR_robot moveit control class"""

    def __init__(self):
        super(UR_robot, self).__init__()
        
        rospy.init_node('robot_node', anonymous=True)
        ## First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). This interface can be used to plan and execute motions:
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        
        ## 设置机器人的最大速度和最大加速度
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.01)
        
        rospy.Subscriber('keyboard_input', String, self.keyboard_callback)

    def go_to_goal_pose(self, pose_goal):
        move_group = self.move_group
        ### Using pose goal method ###
        move_group.clear_pose_targets()

        move_group.set_pose_target(pose_goal)

        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

    def go_to_cartesian_pose(self, pose_goal):
        ### Using catesian path method ###
        move_group = self.move_group
        
        waypoints = []
        waypoints.append(pose_goal)
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        move_group.execute(plan, wait=True)

    def keyboard_callback(self, msg):
        # Set the pose goal orientation to the initial stable orientation
        self.initial_orientation = self.move_group.get_current_pose().pose.orientation
        # Maintain the initial z position
        self.initial_z = self.move_group.get_current_pose().pose.position.z
        
        self.step_size = 0.01
        direction = msg.data
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = self.move_group.get_current_pose().pose
        pose_goal.orientation = current_pose.orientation
        pose_goal.position = current_pose.position

        # if direction == 'w':
        #     pose_goal.position.y += self.step_size
        # elif direction == 's':
        #     pose_goal.position.y -= self.step_size
        # elif direction == 'a':
        #     pose_goal.position.x -= self.step_size
        #     rospy.loginfo("current_pose: {}".format(current_pose.position))
        #     rospy.loginfo("current_orientation: {}".format(current_pose.orientation))
        # elif direction == 'd':
        #     pose_goal.position.x += self.step_size
        #     rospy.loginfo("current_pose: {}".format(current_pose.position))
        #     rospy.loginfo("current_orientation: {}".format(current_pose.orientation))
        # elif direction == 'q':
        #     pose_goal.position.z += self.step_size
        # elif direction == 'e':
        pose_goal.position.z += self.step_size

        rospy.loginfo("pose_goal: {}".format(pose_goal))
        self.go_to_cartesian_pose(pose_goal)
        current_pose = self.move_group.get_current_pose()
        rospy.loginfo("current_pose: {}".format(current_pose))
        
    def run(self):
        rospy.spin()

def main():
    ur_robot = UR_robot()
    ur_robot.run()

if __name__ == "__main__":
    main()