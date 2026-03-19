#!/usr/bin/env python3
"""
person_sensor_node.py

Reads face detection data from the Useful Sensors Person Sensor (I2C address
0x62) and publishes each detected face as a PersonDetection message.

The Person Sensor communicates over I2C and requires no configuration: it
continuously outputs detection results which we read on a timer. The binary
response format is parsed with struct using the formats defined as module-level
constants (derived from the sensor's datasheet).

Topic wiring:
  Publications:
    /person_sensor/detections  (servo_skull_msgs/PersonDetection)
      — one message per face per timer tick (faces below confidence threshold
        are filtered out before publishing)

The node scans all available I2C buses on startup to locate the sensor
automatically, which handles hardware configs where the bus number isn't fixed.
"""
import fcntl
import glob
import io
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from servo_skull_msgs.msg import PersonDetection


# ---------------------------------------------------------------------------
# I2C protocol constants
# ---------------------------------------------------------------------------

# Sensor's fixed I2C address (not configurable)
PERSON_SENSOR_I2C_ADDRESS = 0x62

# ioctl request code to set the I2C target device address
I2C_PERIPHERAL = 0x703

# How often to poll the sensor (seconds). At 0.2 s we get 5 Hz detections.
PERSON_SENSOR_DELAY = 0.2


# ---------------------------------------------------------------------------
# Binary response layout
#
# The sensor writes a fixed-size binary blob that we parse with struct.
# All values are little-endian unsigned unless marked otherwise.
#
# Header (3 bytes):
#   B  padding / reserved byte
#   B  padding / reserved byte
#   H  checksum (uint16)
#
# Face record (8 bytes each, up to PERSON_SENSOR_FACE_MAX records):
#   B  box_confidence    0–255 detection confidence
#   B  box_left         bounding box left edge (0–255, proportional)
#   B  box_top          bounding box top edge
#   B  box_right        bounding box right edge
#   B  box_bottom       bounding box bottom edge
#   B  id_confidence    0–255 confidence in the recognised person ID
#   b  id               recognised person ID (signed; -1 = unknown)
#   B  is_facing        1 if the face is oriented toward the camera
#
# Footer (2 bytes):
#   H  reserved / checksum
# ---------------------------------------------------------------------------

PERSON_SENSOR_I2C_HEADER_FORMAT = "BBH"
PERSON_SENSOR_I2C_HEADER_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_I2C_HEADER_FORMAT)

PERSON_SENSOR_FACE_FORMAT = "BBBBBBbB"
PERSON_SENSOR_FACE_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_FACE_FORMAT)

PERSON_SENSOR_FACE_MAX = 4  # sensor reports at most 4 faces simultaneously

# Full result format: header + face_count byte + N face records + footer uint16
PERSON_SENSOR_RESULT_FORMAT = (
    PERSON_SENSOR_I2C_HEADER_FORMAT
    + "B"
    + PERSON_SENSOR_FACE_FORMAT * PERSON_SENSOR_FACE_MAX
    + "H"
)
PERSON_SENSOR_RESULT_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_RESULT_FORMAT)

# Minimum box_confidence to publish a detection. Below this the bounding box
# is too noisy to be useful for tracking (empirically determined threshold).
CONFIDENCE_THRESHOLD = 80


# ---------------------------------------------------------------------------
# I2C bus discovery
# ---------------------------------------------------------------------------

def find_i2c_bus_for_address(address: int) -> int | None:
    """
    Brute-force scan every /dev/i2c-* bus and return the first bus number on
    which `address` responds. Returns None if the device is not found.

    Uses the I2C_PERIPHERAL ioctl to attempt to address the device; EREMOTEIO
    indicates the address is not present on that bus (normal — keep scanning).
    Any other error is silently ignored (e.g. permission denied).
    """
    import errno
    for dev in sorted(glob.glob("/dev/i2c-*")):
        try:
            handle = io.open(dev, "rb", buffering=0)
            try:
                fcntl.ioctl(handle, I2C_PERIPHERAL, address)
                handle.close()
                return int(dev.split("-")[-1])
            except OSError as e:
                handle.close()
                if e.errno != errno.EREMOTEIO:
                    # Unexpected error; skip this bus but keep scanning
                    pass
        except Exception:
            continue
    return None


# ---------------------------------------------------------------------------
# PersonSensorNode
# ---------------------------------------------------------------------------

class PersonSensorNode(Node):
    def __init__(self):
        super().__init__("person_sensor_node")
        self.publisher_ = self.create_publisher(
            PersonDetection, "/person_sensor/detections", 10
        )
        self.get_logger().info("Person Sensor Node started.")

        self.i2c_handle = self._open_i2c()
        self.timer = self.create_timer(PERSON_SENSOR_DELAY, self.timer_callback)

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _open_i2c(self) -> io.RawIOBase | None:
        """Locate the sensor on any I2C bus and return an open file handle, or None."""
        bus = find_i2c_bus_for_address(PERSON_SENSOR_I2C_ADDRESS)
        if bus is None:
            self.get_logger().error("Person sensor not found on any I2C bus.")
            return None

        self.get_logger().info(f"Found person sensor at I2C bus {bus}.")
        try:
            handle = io.open(f"/dev/i2c-{bus}", "rb", buffering=0)
            fcntl.ioctl(handle, I2C_PERIPHERAL, PERSON_SENSOR_I2C_ADDRESS)
            self.get_logger().info("Person Sensor I2C connection established.")
            return handle
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C device: {e}")
            return None

    # -----------------------------------------------------------------------
    # Timer callback — read and publish
    # -----------------------------------------------------------------------

    def timer_callback(self):
        if not self.i2c_handle:
            self.get_logger().error("I2C handle not available. Skipping read.")
            return

        read_bytes = self._read_sensor()
        if read_bytes is None:
            return

        self._parse_and_publish(read_bytes)

    def _read_sensor(self) -> bytes | None:
        """Read one full result frame from the sensor. Returns None on I2C error."""
        try:
            start_time = time.time()
            data = self.i2c_handle.read(PERSON_SENSOR_RESULT_BYTE_COUNT)
            elapsed = time.time() - start_time
            if elapsed > 0.05:
                self.get_logger().warning(f"I2C read took {elapsed:.3f}s")
            return data
        except OSError as error:
            self.get_logger().warning(f"No person sensor data found: {error}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error during I2C read: {e}")
            return None

    def _parse_and_publish(self, read_bytes: bytes):
        """Unpack the binary sensor response and publish one msg per valid face."""
        offset = 0
        try:
            # Skip the 3-byte header (reserved bytes + checksum)
            struct.unpack_from(PERSON_SENSOR_I2C_HEADER_FORMAT, read_bytes, offset)
            offset += PERSON_SENSOR_I2C_HEADER_BYTE_COUNT

            (num_faces,) = struct.unpack_from("B", read_bytes, offset)
            num_faces = int(num_faces)
            if num_faces > PERSON_SENSOR_FACE_MAX:
                self.get_logger().warning(
                    f"Sensor reported {num_faces} faces; clamping to {PERSON_SENSOR_FACE_MAX}."
                )
                num_faces = PERSON_SENSOR_FACE_MAX
            offset += 1
        except Exception as e:
            self.get_logger().error(f"Failed to parse sensor header: {e}")
            return

        faces_published = 0
        confidences: list[float] = []

        for i in range(num_faces):
            try:
                (
                    box_confidence,
                    box_left, box_top, box_right, box_bottom,
                    id_confidence, face_id, is_facing,
                ) = struct.unpack_from(PERSON_SENSOR_FACE_FORMAT, read_bytes, offset)
                offset += PERSON_SENSOR_FACE_BYTE_COUNT
            except Exception as e:
                self.get_logger().error(f"Failed to parse face {i}: {e}")
                continue

            # Ignore detections where the bounding box confidence is too low
            if box_confidence < CONFIDENCE_THRESHOLD:
                self.get_logger().debug(
                    f"Ignoring face {i}: confidence {box_confidence} < {CONFIDENCE_THRESHOLD}"
                )
                continue

            msg = PersonDetection()
            msg.box_confidence = float(box_confidence)
            msg.box_left       = float(box_left)
            msg.box_top        = float(box_top)
            msg.box_right      = float(box_right)
            msg.box_bottom     = float(box_bottom)
            msg.id_confidence  = float(id_confidence)
            msg.id             = int(face_id)
            msg.is_facing      = bool(is_facing)
            self.publisher_.publish(msg)
            faces_published += 1
            confidences.append(msg.box_confidence)

        if faces_published == 0:
            self.get_logger().debug("No valid faces detected in this frame.")
        else:
            avg_conf = statistics.mean(confidences)
            self.get_logger().info(
                f"Published {faces_published} face(s). Avg confidence: {avg_conf:.1f}"
            )

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def destroy_node(self):
        if self.i2c_handle:
            self.i2c_handle.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PersonSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
