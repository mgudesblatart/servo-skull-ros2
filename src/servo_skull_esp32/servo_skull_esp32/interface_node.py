#!/usr/bin/env python3
"""
interface_node.py

Serial bridge between ROS 2 and the ESP32 microcontroller. Forwards pan/tilt
position commands and LED matrix display commands to the ESP32 over USB serial,
and re-publishes sensor data (ultrasonic distance, info strings, error strings)
received from the ESP32 as ROS topics.

Topic wiring:
  Subscriptions:
    skull_control/pan_tilt_cmd  (geometry_msgs/Point)  — x=pan, y=tilt target positions
    led_matrix/display_cmd      (std_msgs/String)       — display command string
  Publications:
    esp32/ultrasonic  (std_msgs/String)  — distance reading from the ESP32
    esp32/info        (std_msgs/String)  — informational messages from firmware
    esp32/error       (std_msgs/String)  — error messages from firmware

Serial protocol:
  Outbound (ROS → ESP32):
    Pan/tilt : "{x},{y},{checksum}\n"  where checksum = (x + y) & 0xFF
    Display  : "DISPLAY:{cmd}\n"
  Inbound (ESP32 → ROS):
    Lines are prefix-routed: "ULTRASONIC:", "INFO:", "ERROR:"

The port is discovered automatically (first /dev/ttyUSB* found). DTR and RTS
are explicitly lowered after open to prevent the ESP32 from reseting — the
Arduino bootloader treats DTR as a reset signal, so leaving it high causes an
unintended reboot when we connect.

Note: serial_close() exists for explicit teardown, but the main() function
calls it in its finally block. destroy_node() does NOT currently call it,
which means graceful cleanup depends on main() completing the finally path.
"""

import glob
import threading

import rclpy
import serial
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

def find_serial_port() -> str:
    """Return the first /dev/ttyUSB* device found, sorted for determinism."""
    candidates = sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("No /dev/ttyUSB* devices found")
    return candidates[0]


# Module-level port selection happens at import time so it's shared by all
# instances (there's only ever one in practice)
SERIAL_PORT = find_serial_port()
BAUDRATE = 115200


# ---------------------------------------------------------------------------
# ESP32InterfaceNode
# ---------------------------------------------------------------------------

class ESP32InterfaceNode(Node):
    def __init__(self):
        super().__init__("interface_node", enable_logger_service=True)

        self._open_serial()
        self._setup_publishers()
        self._setup_subscriptions()

        # Background thread reads the serial port line-by-line and routes
        # each line to the appropriate ROS publisher
        self.serial_thread = threading.Thread(
            target=self.serial_read_loop, daemon=True
        )
        self.serial_thread.start()
        self.get_logger().info("ESP32 Interface Node started.")

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _open_serial(self):
        """Open the serial port. DTR/RTS are lowered to prevent ESP32 reset."""
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            # Deassert both control lines — the ESP32's CH340 auto-reset circuit
            # triggers a reboot if DTR goes high on connect.
            self.serial.dtr = False
            self.serial.rts = False
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Lock shared between the read loop thread and any callback that writes
        self.serial_lock = threading.Lock()

    def _setup_publishers(self):
        self.ultrasonic_pub = self.create_publisher(String, "esp32/ultrasonic", 10)
        self.info_pub = self.create_publisher(String, "esp32/info", 10)
        self.error_pub = self.create_publisher(String, "esp32/error", 10)

    def _setup_subscriptions(self):
        self.xy_sub = self.create_subscription(
            Point, "skull_control/pan_tilt_cmd", self.xy_cmd_callback, 10
        )
        self.display_sub = self.create_subscription(
            String, "led_matrix/display_cmd", self.display_cmd_callback, 10
        )

    # -----------------------------------------------------------------------
    # Serial read loop (background thread)
    # -----------------------------------------------------------------------

    def serial_read_loop(self):
        """
        Continuously read characters from the serial port and split on newlines.
        Lines are passed to handle_serial_line for prefix-based routing.
        Runs as long as the ROS context is active.
        """
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

    def handle_serial_line(self, line: str):
        """
        Route a single decoded serial line to the correct ROS topic based on
        its prefix. Unrecognised lines are logged as warnings.
        """
        if line.startswith("ULTRASONIC:"):
            msg = String()
            msg.data = line[len("ULTRASONIC:"):]
            self.ultrasonic_pub.publish(msg)
        elif line.startswith("INFO:"):
            msg = String()
            msg.data = line[len("INFO:"):]
            self.info_pub.publish(msg)
        elif line.startswith("ERROR:"):
            msg = String()
            msg.data = line[len("ERROR:"):]
            self.error_pub.publish(msg)
        else:
            self.get_logger().warning(f"Unknown serial message: {line}")

    # -----------------------------------------------------------------------
    # Topic callbacks (ROS executor thread)
    # -----------------------------------------------------------------------

    def xy_cmd_callback(self, msg: Point):
        """
        Send pan/tilt coordinates to the ESP32.
        Checksum = (x + y) & 0xFF — simple single-byte integrity check that
        the ESP32 firmware validates before acting on the command.
        """
        x = int(msg.x)
        y = int(msg.y)
        checksum = (x + y) & 0xFF
        out_str = f"{x},{y},{checksum}\n"
        try:
            self.serial.reset_input_buffer()  # discard stale inbound data
            with self.serial_lock:
                self.serial.write(out_str.encode())
            self.get_logger().debug(f"Sent to ESP32: {out_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def display_cmd_callback(self, msg: String):
        """Forward a display command string to the ESP32's LED matrix handler."""
        out_str = f"DISPLAY:{msg.data}\n"
        try:
            with self.serial_lock:
                self.serial.write(out_str.encode("utf-8"))
            self.get_logger().debug(f"Sent to ESP32: {out_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def serial_close(self):
        """Close the serial port. Called explicitly from main()'s finally block."""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.get_logger().info("Serial port closed.")
        except Exception as e:
            self.get_logger().error(f"Error closing serial port: {e}")

    def destroy_node(self):
        # Ensure serial is always closed even if callers skip explicit serial_close().
        self.serial_close()
        return super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ESP32InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
        rclpy.shutdown()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
