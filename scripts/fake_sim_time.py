#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

class SimTimePublisher(Node):
    def __init__(self):
        super().__init__('sim_time_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.simulated_time = 0.0
        self.timer = self.create_timer(0.1, self.publish_sim_time)

    def publish_sim_time(self):
        self.simulated_time += 0.1
        clock_msg = Clock()
        clock_msg.clock.sec = int(self.simulated_time)
        clock_msg.clock.nanosec = int((self.simulated_time - int(self.simulated_time)) * 1e9)
        self.publisher_.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimTimePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
