#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from servo_skull_msgs.msg import TrackedPersons, TrackedPerson
import time
from geometry_msgs.msg import Point

SENSOR_MIN = 0
SENSOR_MAX = 255
ESP32_MIN = 0
ESP32_MAX = 500
TARGET_TIMOUT_SEC = 5  # configurable sticky timeout

class SkullControlNode(Node):
    def __init__(self):
        super().__init__('skull_control_node')
        self.subscription = self.create_subscription(
            TrackedPersons,
            '/person_tracking/tracked_persons',
            self.tracked_persons_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Point, 'skull_control/pan_tilt_cmd', 10)
        self.current_target = None
        self.current_target_id = None
        self.target_lost_time = None
        self.target_timeout_sec = TARGET_TIMOUT_SEC  # configurable sticky timeout
        self.get_logger().info('Skull Control Node started. Subscribed to /person_tracking/tracked_persons')

    def tracked_persons_callback(self, msg: TrackedPersons):
        now = time.monotonic()
        tracks = msg.tracks
        if not tracks:
            if self.current_target_id is not None and self.target_lost_time is None:
                self.target_lost_time = now
                self.get_logger().info('Target lost, starting timeout.')
            elif self.target_lost_time is not None and (now - self.target_lost_time) > self.target_timeout_sec:
                self.get_logger().info('Target timeout expired. Clearing target.')
                self.current_target = None
                self.current_target_id = None
                self.target_lost_time = None
            return

        # Try to find the current target in the new tracks
        found = None
        for p in tracks:
            if p.person_id == self.current_target_id:
                found = p
                break
        if found:
            self.current_target = found
            self.target_lost_time = None
            self.get_logger().info(f'Sticky target: id={found.person_id}, conf={found.box_confidence:.2f}, state={found.state}')
        else:
            # Select new target (highest confidence)
            best = max(tracks, key=lambda p: p.box_confidence)
            self.current_target = best
            self.current_target_id = best.person_id
            self.target_lost_time = None
            self.get_logger().info(f'New target selected: id={best.person_id}, conf={best.box_confidence:.2f}, state={best.state}')

        # Convert bounding box to X,Y (use center, scale from sensor to ESP32 range)
        if self.current_target:
            x_raw = int((self.current_target.box_left + self.current_target.box_right) / 2)
            y_raw = int((self.current_target.box_top + self.current_target.box_bottom) / 2)
            x = int((x_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            y = int((y_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            self.cmd_pub.publish(pt)
            self.get_logger().debug(f'Published pan/tilt: x={x}, y={y} (raw: {x_raw}, {y_raw})')


def main(args=None):
    rclpy.init(args=args)
    node = SkullControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
