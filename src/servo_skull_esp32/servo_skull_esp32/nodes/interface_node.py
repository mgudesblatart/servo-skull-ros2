#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Point

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


class ESP32InterfaceNode(Node):
    def __init__(self):
        super().__init__("interface_node", enable_logger_service=True)
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise
        self.serial_lock = threading.Lock()
        self.ultrasonic_pub = self.create_publisher(String, "esp32/ultrasonic", 10)
        self.info_pub = self.create_publisher(String, "esp32/info", 10)
        self.error_pub = self.create_publisher(String, "esp32/error", 10)
        self.xy_sub = self.create_subscription(
            Point, "skull_control/pan_tilt_cmd", self.xy_cmd_callback, 10
        )
        self.display_sub = self.create_subscription(
            String, "led_matrix/display_cmd", self.display_cmd_callback, 10
        )
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()
        self.get_logger().info("ESP32 Interface Node started.")

    def serial_read_loop(self):
        buffer = ""
        while rclpy.ok():
            try:
                with self.serial_lock:
                    data = self.serial.read(self.serial.in_waiting or 1).decode()
                if data:
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        self.handle_serial_line(line.strip())
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

    def handle_serial_line(self, line):
        self.get_logger().debug(f"Received serial line: {line}")
        if line.startswith("ULTRASONIC:"):
            msg = String()
            msg.data = line[len("ULTRASONIC:") :]
            self.ultrasonic_pub.publish(msg)
        elif line.startswith("INFO:"):
            msg = String()
            msg.data = line[len("INFO:") :]
            self.info_pub.publish(msg)
        elif line.startswith("ERROR:"):
            msg = String()
            msg.data = line[len("ERROR:") :]
            self.error_pub.publish(msg)
        else:
            self.get_logger().warning(f"Unknown serial message: {line}")

    def xy_cmd_callback(self, msg):
        self.get_logger().debug(f"Received XY command: {msg.x}, {msg.y}")
        x = int(msg.x)
        y = int(msg.y)
        checksum = (x + y) & 0xFF
        out_str = f"{x},{y},{checksum}\n"
        try:
            self.serial.reset_input_buffer()
            with self.serial_lock:
                self.serial.write(out_str.encode())
            self.get_logger().debug(f"Sent to ESP32: {out_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def display_cmd_callback(self, msg):
        out_str = f"DISPLAY:{msg.data}\n"
        try:
            with self.serial_lock:
                self.serial.write(out_str.encode("utf-8"))
            self.get_logger().debug(f"Sent to ESP32: {out_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def serial_close(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.get_logger().info("Serial port closed.")
        except Exception as e:
            self.get_logger().error(f"Error closing serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ESP32InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down node.')
        rclpy.shutdown()
    finally:
        node.serial_close()
        node.destroy_node()


if __name__ == "__main__":
    main()
