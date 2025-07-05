#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from servo_skull_msgs.msg import PersonDetection
import io
import fcntl
import struct
import time
import statistics

PERSON_SENSOR_I2C_ADDRESS = 0x62
PERSON_SENSOR_I2C_HEADER_FORMAT = "BBH"
PERSON_SENSOR_I2C_HEADER_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_I2C_HEADER_FORMAT)
PERSON_SENSOR_FACE_FORMAT = "BBBBBBbB"
PERSON_SENSOR_FACE_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_FACE_FORMAT)
PERSON_SENSOR_FACE_MAX = 4
PERSON_SENSOR_RESULT_FORMAT = PERSON_SENSOR_I2C_HEADER_FORMAT + "B" + PERSON_SENSOR_FACE_FORMAT * PERSON_SENSOR_FACE_MAX + "H"
PERSON_SENSOR_RESULT_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_RESULT_FORMAT)
I2C_CHANNEL = 1
I2C_PERIPHERAL = 0x703
PERSON_SENSOR_DELAY = 0.2

class PersonSensorNode(Node):
    def __init__(self):
        super().__init__('person_sensor_node')
        self.publisher_ = self.create_publisher(PersonDetection, '/person_sensor/detections', 10)
        self.get_logger().info('Person Sensor Node started.')
        try:
            self.i2c_handle = io.open(f"/dev/i2c-{I2C_CHANNEL}", "rb", buffering=0)
            fcntl.ioctl(self.i2c_handle, I2C_PERIPHERAL, PERSON_SENSOR_I2C_ADDRESS)
            self.get_logger().info('Person Sensor I2C connection established.')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C device: {e}')
            self.i2c_handle = None
        self.timer = self.create_timer(PERSON_SENSOR_DELAY, self.timer_callback)

    def timer_callback(self):
        if not self.i2c_handle:
            self.get_logger().error('I2C handle not available. Skipping read.')
            return
        try:
            start_time = time.time()
            read_bytes = self.i2c_handle.read(PERSON_SENSOR_RESULT_BYTE_COUNT)
            elapsed = time.time() - start_time
            if elapsed > 0.05:
                self.get_logger().warning(f'I2C read took {elapsed:.3f}s')
        except OSError as error:
            self.get_logger().warning(f"No person sensor data found: {error}")
            self.sensor_health = False
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected error during I2C read: {e}")
            self.sensor_health = False
            return
        self.sensor_health = True
        offset = 0
        try:
            _, _, _ = struct.unpack_from(PERSON_SENSOR_I2C_HEADER_FORMAT, read_bytes, offset)
            offset += PERSON_SENSOR_I2C_HEADER_BYTE_COUNT
            (num_faces,) = struct.unpack_from("B", read_bytes, offset)
            num_faces = int(num_faces)
            offset += 1
        except Exception as e:
            self.get_logger().error(f"Failed to parse sensor header: {e}")
            return
        faces_published = 0
        confidences = []
        for i in range(num_faces):
            try:
                (box_confidence, box_left, box_top, box_right, box_bottom, id_confidence, id, is_facing) = struct.unpack_from(PERSON_SENSOR_FACE_FORMAT, read_bytes, offset)
                offset += PERSON_SENSOR_FACE_BYTE_COUNT
                self.get_logger().info(f"Face {i}: conf={box_confidence}, left={box_left}, top={box_top}, right={box_right}, bottom={box_bottom}, id_confidence={id_confidence}, id={id}, facing={is_facing}")
                # Data filtering: ignore faces with low confidence
                if box_confidence < 90:
                    self.get_logger().info(f"Ignoring face {i} with low confidence: {box_confidence}")
                    continue
                msg = PersonDetection()
                msg.box_confidence = float(box_confidence)
                msg.box_left = float(box_left)
                msg.box_top = float(box_top)
                msg.box_right = float(box_right)
                msg.box_bottom = float(box_bottom)
                msg.id_confidence = float(id_confidence)
                msg.id = int(id)
                msg.is_facing = bool(is_facing)
                self.publisher_.publish(msg)
                faces_published += 1
                confidences.append(msg.box_confidence)
                self.get_logger().info(f'Published: id={msg.id}, conf={msg.box_confidence:.2f}, facing={msg.is_facing}')
            except Exception as e:
                self.get_logger().error(f"Failed to parse face {i}: {e}")
        if faces_published == 0:
            self.get_logger().info('No valid faces detected in this frame.')
        else:
            avg_conf = statistics.mean(confidences) if confidences else 0.0
            self.get_logger().info(f"Published {faces_published} faces. Avg confidence: {avg_conf:.2f}")

    def destroy_node(self):
        if self.i2c_handle:
            self.i2c_handle.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PersonSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
