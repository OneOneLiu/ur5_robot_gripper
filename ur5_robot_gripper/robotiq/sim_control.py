#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
from rclpy.duration import Duration

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control_node')

        # ROS initialization
        self.publisher = self.create_publisher(FollowJointTrajectory.Goal, 
                                               '/gripper_controller/follow_joint_trajectory/goal', 10)

        # GUI setup
        self.root = tk.Tk()
        self.root.title("Gripper Control")
        
        ttk.Label(self.root, text="Gripper Position:").pack()
        self.slider = ttk.Scale(self.root, from_=0.0, to=0.8, orient='horizontal', 
                                length=300,  # Make the slider longer
                                command=self.control_gripper)
        self.slider.pack(fill=tk.X, expand=True)  # Add some padding around the slider

    def control_gripper(self, open_width):
        """Control the gripper position.
        @open_width: the width of the gripper opening in the range [0, 0.8].
        """
        # Prepare the message
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"] 
        point = JointTrajectoryPoint()
        point.positions = [float(open_width)]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        msg.trajectory.points.append(point)
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent gripper command: {float(open_width)}")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    app = GripperControl()
    app.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
