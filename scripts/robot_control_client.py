#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur5_robot_gripper.srv import MoveToPosition

class RobotClientNode(Node):
    def __init__(self):
        super().__init__('robot_client_node')
        self.client = self.create_client(MoveToPosition, 'move_to_position')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service available, sending request...')
        self.send_request(0.5, 0.1, 0.2)  # Example coordinates

    def send_request(self, px, py, pz):
        request = MoveToPosition.Request()
        request.px = px
        request.py = py
        request.pz = pz

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot moved to the desired position successfully.')
            else:
                self.get_logger().info('Failed to move the robot to the desired position.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    robot_client_node = RobotClientNode()
    rclpy.spin(robot_client_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
