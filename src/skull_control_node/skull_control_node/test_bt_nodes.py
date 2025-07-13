import unittest
import py_trees
from geometry_msgs.msg import Point

# Mock classes for testing
class MockPublisher:
    def __init__(self):
        self.published = []
    def publish(self, msg):
        self.published.append(msg)

class MockPerson:
    def __init__(self, person_id, box_confidence, box_left, box_right, box_top, box_bottom):
        self.person_id = person_id
        self.box_confidence = box_confidence
        self.box_left = box_left
        self.box_right = box_right
        self.box_top = box_top
        self.box_bottom = box_bottom

from skull_control_bt_node import Blackboard, TrackSelector, PanTiltPublisher, SENSOR_MIN, SENSOR_MAX, ESP32_MIN, ESP32_MAX

class TestBehaviorTreeNodes(unittest.TestCase):
    def setUp(self):
        self.blackboard = Blackboard()
        self.publisher = MockPublisher()

    def test_track_selector_no_tracks(self):
        selector = TrackSelector('TrackSelector', self.blackboard)
        status = selector.update()
        self.assertEqual(status, py_trees.common.Status.FAILURE)
        self.assertIsNone(self.blackboard.current_target)

    def test_track_selector_selects_best(self):
        p1 = MockPerson(1, 0.5, 10, 20, 30, 40)
        p2 = MockPerson(2, 0.9, 50, 60, 70, 80)
        self.blackboard.tracks = [p1, p2]
        selector = TrackSelector('TrackSelector', self.blackboard)
        status = selector.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)
        self.assertEqual(self.blackboard.current_target, p2)
        self.assertEqual(self.blackboard.current_target_id, 2)

    def test_track_selector_sticky_target(self):
        p1 = MockPerson(1, 0.8, 10, 20, 30, 40)
        self.blackboard.tracks = [p1]
        self.blackboard.current_target_id = 1
        selector = TrackSelector('TrackSelector', self.blackboard)
        status = selector.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)
        self.assertEqual(self.blackboard.current_target, p1)

    def test_pan_tilt_publisher_success(self):
        p = MockPerson(1, 0.8, 0, 255, 0, 255)
        self.blackboard.current_target = p
        pan_tilt = PanTiltPublisher('PanTiltPublisher', self.blackboard, self.publisher)
        status = pan_tilt.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)
        self.assertEqual(len(self.publisher.published), 1)
        pt = self.publisher.published[0]
        self.assertIsInstance(pt, Point)
        self.assertAlmostEqual(pt.x, ESP32_MAX / 2, delta=1)
        self.assertAlmostEqual(pt.y, ESP32_MAX / 2, delta=1)

    def test_pan_tilt_publisher_failure(self):
        pan_tilt = PanTiltPublisher('PanTiltPublisher', self.blackboard, self.publisher)
        status = pan_tilt.update()
        self.assertEqual(status, py_trees.common.Status.FAILURE)
        self.assertEqual(len(self.publisher.published), 0)

if __name__ == '__main__':
    unittest.main()
