import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import unittest
from person_sensor_node.person_sensor_node import PersonSensorNode, PERSON_SENSOR_FACE_FORMAT
import struct

class DummyPublisher:
    def __init__(self):
        self.published = []
    def publish(self, msg):
        self.published.append(msg)

class TestPersonSensorNode(unittest.TestCase):
    def setUp(self):
        self.node = PersonSensorNode()
        self.node.publisher_ = DummyPublisher()  # Patch publisher

    def test_decode_single_face(self):
        # Simulate a single face detection with known values
        box_confidence = 200
        box_left = 50
        box_top = 60
        box_right = 180
        box_bottom = 200
        id_confidence = 180
        id = 3
        is_facing = 1
        face_bytes = struct.pack(PERSON_SENSOR_FACE_FORMAT, box_confidence, box_left, box_top, box_right, box_bottom, id_confidence, id, is_facing)
        # Compose a fake I2C read buffer
        header = struct.pack('BBH', 0, 0, 0)
        num_faces = struct.pack('B', 1)
        checksum = struct.pack('H', 0)
        read_bytes = header + num_faces + face_bytes + b'\x00' * (self.node.PERSON_SENSOR_FACE_BYTE_COUNT * (self.node.PERSON_SENSOR_FACE_MAX - 1)) + checksum
        # Patch i2c_handle.read to return this buffer
        self.node.i2c_handle = type('', (), {'read': lambda s, n: read_bytes})()
        self.node.timer_callback()
        self.assertEqual(len(self.node.publisher_.published), 1)
        msg = self.node.publisher_.published[0]
        self.assertAlmostEqual(msg.box_confidence, box_confidence / 255.0)
        self.assertEqual(msg.id, id)
        self.assertTrue(msg.is_facing)

if __name__ == '__main__':
    unittest.main()
