import struct
import unittest
from person_sensor_node.person_sensor_node import (
    PERSON_SENSOR_FACE_BYTE_COUNT,
    PERSON_SENSOR_FACE_FORMAT,
    PERSON_SENSOR_FACE_MAX,
    PersonSensorNode
)
import rclpy


class DummyPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class TestPersonSensorNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = PersonSensorNode()
        self.node.publisher_ = DummyPublisher()  # Patch publisher

    def tearDown(self):
        rclpy.shutdown()

    def test_decode_single_face(self):
        # Simulate a single face detection with known values
        box_confidence = 200
        box_left = 50
        box_top = 60
        box_right = 180
        box_bottom = 200
        id_confidence = 180
        person_id = 3
        is_facing = 1
        face_bytes = struct.pack(
            PERSON_SENSOR_FACE_FORMAT,
            box_confidence, box_left, box_top, box_right, box_bottom,
            id_confidence, person_id, is_facing
        )
        header = struct.pack('BBH', 0, 0, 0)
        num_faces = struct.pack('B', 1)
        checksum = struct.pack('H', 0)
        read_bytes = (
            header + num_faces + face_bytes +
            b'\x00' * (PERSON_SENSOR_FACE_BYTE_COUNT * (PERSON_SENSOR_FACE_MAX - 1)) +
            checksum
        )
        self.node.i2c_handle = type('', (), {'read': lambda s, n: read_bytes})()
        self.node.timer_callback()
        self.assertEqual(len(self.node.publisher_.published), 1)
        msg = self.node.publisher_.published[0]
        self.assertEqual(msg.box_confidence, box_confidence)
        self.assertEqual(msg.id, person_id)
        self.assertTrue(msg.is_facing)

    def test_decode_low_confidence_face(self):
        # Simulate a face with low confidence (should be ignored)
        box_confidence = 50  # Below threshold of 90
        face_bytes = struct.pack(
            PERSON_SENSOR_FACE_FORMAT, box_confidence, 10, 10, 20, 20, 100, 1, 1
        )
        header = struct.pack('BBH', 0, 0, 0)
        num_faces = struct.pack('B', 1)
        checksum = struct.pack('H', 0)
        read_bytes = (
            header + num_faces + face_bytes +
            b'\x00' * (PERSON_SENSOR_FACE_BYTE_COUNT * (PERSON_SENSOR_FACE_MAX - 1)) +
            checksum
        )
        self.node.i2c_handle = type('', (), {'read': lambda s, n: read_bytes})()
        self.node.publisher_.published.clear()
        self.node.timer_callback()
        self.assertEqual(len(self.node.publisher_.published), 0)

    def test_i2c_error_handling(self):
        # Simulate an OSError during I2C read
        class FailingI2C:
            def read(self, n):
                raise OSError('Simulated I2C failure')
        self.node.i2c_handle = FailingI2C()
        self.node.publisher_.published.clear()
        self.node.timer_callback()
        self.assertEqual(len(self.node.publisher_.published), 0)


if __name__ == '__main__':
    unittest.main()
