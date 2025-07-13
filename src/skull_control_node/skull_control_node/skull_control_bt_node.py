#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from servo_skull_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import Point
import time

SENSOR_MIN = 0
SENSOR_MAX = 255
ESP32_MIN = 0
ESP32_MAX = 500
TARGET_TIMEOUT_SEC = 5

class Blackboard:
    def __init__(self):
        self.current_target = None
        self.current_target_id = None
        self.target_lost_time = None
        self.tracks = []

class TrackSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super().__init__(name)
        self.blackboard = blackboard

    def update(self):
        tracks = self.blackboard.tracks
        now = time.monotonic()
        if not tracks:
            if self.blackboard.current_target_id is not None and self.blackboard.target_lost_time is None:
                self.blackboard.target_lost_time = now
            elif self.blackboard.target_lost_time is not None and (now - self.blackboard.target_lost_time) > TARGET_TIMEOUT_SEC:
                self.blackboard.current_target = None
                self.blackboard.current_target_id = None
                self.blackboard.target_lost_time = None
            return py_trees.common.Status.FAILURE
        found = None
        for p in tracks:
            if p.person_id == self.blackboard.current_target_id:
                found = p
                break
        if found:
            self.blackboard.current_target = found
            self.blackboard.target_lost_time = None
        else:
            best = max(tracks, key=lambda p: p.box_confidence)
            self.blackboard.current_target = best
            self.blackboard.current_target_id = best.person_id
            self.blackboard.target_lost_time = None
        return py_trees.common.Status.SUCCESS

class PanTiltPublisher(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, publisher):
        super().__init__(name)
        self.blackboard = blackboard
        self.publisher = publisher

    def update(self):
        if self.blackboard.current_target:
            x_raw = int((self.blackboard.current_target.box_left + self.blackboard.current_target.box_right) / 2)
            y_raw = int((self.blackboard.current_target.box_top + self.blackboard.current_target.box_bottom) / 2)
            x = int((x_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            y = int((y_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            self.publisher.publish(pt)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class SkullControlBTNode(Node):
    def __init__(self):
        super().__init__('skull_control_bt_node')
        self.blackboard = Blackboard()
        self.subscription = self.create_subscription(
            TrackedPersons,
            '/person_tracking/tracked_persons',
            self.tracked_persons_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Point, 'skull_control/pan_tilt_cmd', 10)
        # Build behavior tree
        selector = TrackSelector('TrackSelector', self.blackboard)
        pan_tilt = PanTiltPublisher('PanTiltPublisher', self.blackboard, self.cmd_pub)
        sequence = py_trees.composites.Sequence('MainSequence')
        sequence.add_children([selector, pan_tilt])
        self.tree = py_trees_ros.trees.BehaviourTree(sequence)
        self.get_logger().info('Skull Control BT Node started.')

    def tracked_persons_callback(self, msg: TrackedPersons):
        self.blackboard.tracks = msg.tracks

    def spin(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            self.tree.tick()
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = SkullControlBTNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
