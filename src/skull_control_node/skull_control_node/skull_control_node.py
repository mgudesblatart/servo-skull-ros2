#!/usr/bin/env python3
"""
skull_control_node.py

Lightweight, standalone pan/tilt controller — no FSM, no LLM wiring.
Subscribes to tracked person data, picks a target, and publishes servo
commands. Exists as a simpler alternative to skull_control_bt_node.py
for hardware-only testing.

Topic wiring:
  Subscribed:  /person_tracking/tracked_persons
  Published:   skull_control/pan_tilt_cmd
"""

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from servo_skull_msgs.msg import TrackedPersons

# ---------------------------------------------------------------------------
# Coordinate mapping: person-sensor pixel space -> ESP32 servo range
# ---------------------------------------------------------------------------
SENSOR_MIN = 0
SENSOR_MAX = 255
ESP32_MIN = 0
ESP32_MAX = 500

# How long (seconds) to hold a "sticky" lock on a target after it disappears
TARGET_TIMEOUT_SEC = 5


class SkullControlNode(Node):
    def __init__(self):
        super().__init__('skull_control_node')

        self.subscription = self.create_subscription(
            TrackedPersons,
            '/person_tracking/tracked_persons',
            self.tracked_persons_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Point, 'skull_control/pan_tilt_cmd', 10)

        # Sticky target tracking: prefer the same person across frames
        self.current_target = None
        self.current_target_id = None
        self.target_lost_time = None

        self.get_logger().info('Skull Control Node started.')

    def tracked_persons_callback(self, msg: TrackedPersons):
        now = time.monotonic()
        tracks = msg.tracks

        if not tracks:
            # Start or advance the lost-target timeout
            if self.current_target_id is not None and self.target_lost_time is None:
                self.target_lost_time = now
                self.get_logger().info('Target lost; starting timeout.')
            elif self.target_lost_time is not None and (now - self.target_lost_time) > TARGET_TIMEOUT_SEC:
                self.get_logger().info('Target timeout expired. Clearing target.')
                self.current_target = None
                self.current_target_id = None
                self.target_lost_time = None
            return

        # Try to re-acquire the previously tracked person
        found = next((p for p in tracks if p.person_id == self.current_target_id), None)

        if found:
            self.current_target = found
            self.target_lost_time = None
            self.get_logger().info(
                f'Sticky target: id={found.person_id}, conf={found.box_confidence:.2f}'
            )
        else:
            # Current target not in frame — pick the highest-confidence detection
            best = max(tracks, key=lambda p: p.box_confidence)
            self.current_target = best
            self.current_target_id = best.person_id
            self.target_lost_time = None
            self.get_logger().info(
                f'New target: id={best.person_id}, conf={best.box_confidence:.2f}'
            )

        if self.current_target:
            self._publish_pan_tilt(self.current_target)

    def _publish_pan_tilt(self, target):
        """Convert bounding-box center to servo coordinates and publish."""
        x_raw = int((target.box_left + target.box_right) / 2)
        y_raw = int((target.box_top + target.box_bottom) / 2)

        # Linear remap from sensor pixel space to ESP32 servo space
        x = int((x_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
        y = int((y_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)

        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.0
        self.cmd_pub.publish(pt)
        self.get_logger().debug(f'Pan/tilt cmd: x={x}, y={y} (raw: {x_raw}, {y_raw})')


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

