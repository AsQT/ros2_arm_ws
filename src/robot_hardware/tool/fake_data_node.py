#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class FakeDataNode(Node):
    def __init__(self):
        super().__init__("fake_data_node")

        self.pub_pos = self.create_publisher(
            Float64MultiArray,
            "/robot_hardware/cmd_pos",
            10
        )

        self.pub_vel = self.create_publisher(
            Float64MultiArray,
            "/robot_hardware/cmd_vel",
            10
        )

        self.t = 0.0
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50 Hz

    def timer_cb(self):
        msg = Float64MultiArray()

        # fake 6 joints: sin wave
        msg.data = [
            0.5 * math.sin(self.t + i)
            for i in range(6)
        ]

        self.pub_pos.publish(msg)
        self.t += 0.02


def main():
    rclpy.init()
    node = FakeDataNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
