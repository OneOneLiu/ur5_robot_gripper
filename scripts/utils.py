#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from ur5_robot_gripper.action import MoveToPositionAction, MoveToPoseAction, MoveGripperAction, ExecuteGrasp, MoveToJointPosition
from ur5_robot_gripper.srv import PrintPose
from rclpy.action import ActionClient, ActionServer
import time
import numpy as np
import math
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.axangles import axangle2mat
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException

def construct_pose(position, quaternion):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = quaternion[0]
    pose.orientation.x = quaternion[1]
    pose.orientation.y = quaternion[2]
    pose.orientation.z = quaternion[3]
    return pose

def pose_to_matrix(pose):
    # 提取平移
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])

    # 提取四元数并转换为旋转矩阵
    quaternion = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    rotation_matrix = quat2mat(quaternion)

    # 构建4x4的变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    return transform_matrix

def apply_transform(robot_pose, relative_transform):
    # 将机器人的姿态转换为矩阵
    robot_matrix = pose_to_matrix(robot_pose)

    # 计算新的机器人姿态矩阵
    new_robot_matrix = robot_matrix.dot(relative_transform)

    # 提取新的平移和旋转
    new_translation = new_robot_matrix[:3, 3]
    new_rotation_matrix = new_robot_matrix[:3, :3]
    
    # 转换为四元数
    from transforms3d.quaternions import mat2quat
    new_quaternion = mat2quat(new_rotation_matrix)

    # 构建新的 Pose
    new_pose = Pose()
    new_pose.position.x = new_translation[0]
    new_pose.position.y = new_translation[1]
    new_pose.position.z = new_translation[2]
    new_pose.orientation.w = new_quaternion[0]
    new_pose.orientation.x = new_quaternion[1]
    new_pose.orientation.y = new_quaternion[2]
    new_pose.orientation.z = new_quaternion[3]

    return new_pose