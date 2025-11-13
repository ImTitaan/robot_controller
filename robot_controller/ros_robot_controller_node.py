# import rclpy

"""
Robot Controller Node

Responsible for underlying robot controller logic I/O between the SDK project and
user interactions.

CODY W.

11/5/2025
"""
import math
import rclpy

from rclpy.node import Node
from robot_controller.ros_robot_controller_sdk import Board

from geometry_msgs.msg import Twist


class RobotControllerNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.board = Board()
        self.board.enable_reception()

        # TODO: Double Check Constants
        self.wheelbase = 0.145
        self.track_width = 0.133
        self.wheel_diameter = 0.067

        self.cmd_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.set_motor, 10
        )

    def speed_covert(self, speed):
        return speed / (math.pi * self.wheel_diameter)

    def set_motor(self, msg):
        bounded_x = msg.linear.y if abs(msg.linear.x) <= 1.0 else 1.0
        bounded_y = msg.linear.y if abs(msg.linear.y) <= 1.0 else 1.0
        bounded_z = msg.angular.x if abs(msg.angular.x) <= 1.0 else 1.0

        motor1 = (
            bounded_x - bounded_y - bounded_z * (self.wheelbase + self.track_width) / 2
        )
        motor2 = (
            bounded_x + bounded_y - bounded_z * (self.wheelbase + self.track_width) / 2
        )
        motor3 = (
            bounded_x + bounded_y + bounded_z * (self.wheelbase + self.track_width) / 2
        )
        motor4 = (
            bounded_x - bounded_y + bounded_z * (self.wheelbase + self.track_width) / 2
        )
        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        data = []
        for i in range(len(v_s)):
            id = i + 1
            rps = float(v_s[i])
            data.append([id, rps])
        self.board.set_motor_speed(data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode("robot_controller")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()
        print("shutdown")
    finally:
        print("shutdown finish")


if __name__ == "__main__":
    main()
