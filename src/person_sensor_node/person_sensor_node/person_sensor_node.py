import rclpy
from rclpy.node import Node
from person_sensor_node.msg import PersonDetection
import io
import fcntl
import struct
import time

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
            return
        try:
            read_bytes = self.i2c_handle.read(PERSON_SENSOR_RESULT_BYTE_COUNT)
        except OSError as error:
            self.get_logger().warning(f"No person sensor data found: {error}")
            return
        offset = 0
        _, _, _ = struct.unpack_from(PERSON_SENSOR_I2C_HEADER_FORMAT, read_bytes, offset)
        offset += PERSON_SENSOR_I2C_HEADER_BYTE_COUNT
        (num_faces,) = struct.unpack_from("B", read_bytes, offset)
        num_faces = int(num_faces)
        offset += 1
        for i in range(num_faces):
            (box_confidence, box_left, box_top, box_right, box_bottom, id_confidence, id, is_facing) = struct.unpack_from(PERSON_SENSOR_FACE_FORMAT, read_bytes, offset)
            offset += PERSON_SENSOR_FACE_BYTE_COUNT
            msg = PersonDetection()
            msg.box_confidence = float(box_confidence) / 255.0
            msg.box_left = float(box_left) / 255.0
            msg.box_top = float(box_top) / 255.0
            msg.box_right = float(box_right) / 255.0
            msg.box_bottom = float(box_bottom) / 255.0
            msg.id_confidence = float(id_confidence) / 255.0
            msg.id = int(id)
            msg.is_facing = bool(is_facing)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: id={msg.id}, conf={msg.box_confidence:.2f}, facing={msg.is_facing}')

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
