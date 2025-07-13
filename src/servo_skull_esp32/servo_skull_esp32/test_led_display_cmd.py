#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class LEDDisplayTestNode(Node):
    def __init__(self):
        super().__init__('led_display_test_node')
        self.pub = self.create_publisher(String, 'led_matrix/display_cmd', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.test_msgs = ['HAPPY', 'SAD', 'IDLE', 'TRACKING', 'ERROR']
        self.index = 0
        self.sent = 0

    def timer_callback(self):
        if self.sent >= len(self.test_msgs):
            self.get_logger().info('All test messages sent. Shutting down.')
            rclpy.shutdown()
            return
        msg = String()
        msg.data = self.test_msgs[self.index]
        self.pub.publish(msg)
        self.get_logger().info(f'Sent display command: {msg.data}')
        self.index = (self.index + 1) % len(self.test_msgs)
        self.sent += 1

def main(args=None):
    rclpy.init(args=args)
    node = LEDDisplayTestNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
