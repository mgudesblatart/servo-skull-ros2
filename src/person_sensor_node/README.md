# person_sensor_node

Python ROS2 node. Reads face detection data from a [Useful Sensors Person Sensor](https://usefulsensors.com/person-sensor/) over I2C and publishes `PersonDetection` messages.

---

## Node: `person_sensor_node`

**Pipeline position:** **Person Sensor** → Person Tracking → Skull Control

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Publish | `/person_sensor/detections` | `servo_skull_msgs/PersonDetection` | Per-face detections, polled at ~5 Hz |

**Poll rate:** 200ms timer (`PERSON_SENSOR_DELAY = 0.2`)

---

## Hardware

**Sensor:** Useful Sensors Person Sensor (I2C address `0x62`)

The node auto-detects the correct I2C bus by scanning all `/dev/i2c-*` devices and attempting `ioctl` with the sensor address. No manual bus configuration needed.

**Wiring (Pi 5):** Connect SDA/SCL to any available I2C bus. Enable I2C in `raspi-config` if not already active.

**Permissions:** User must be in the `i2c` group:
```bash
sudo usermod -aG i2c $USER  # re-login required
```

---

## Message Format

Each `PersonDetection` message corresponds to one detected face from the sensor:

| Field | Type | Description |
|---|---|---|
| `box_confidence` | `float32` | Detection confidence [0.0–1.0] |
| `box_left` | `float32` | Left edge of bounding box [0–255] |
| `box_top` | `float32` | Top edge of bounding box [0–255] |
| `box_right` | `float32` | Right edge of bounding box [0–255] |
| `box_bottom` | `float32` | Bottom edge of bounding box [0–255] |
| `id_confidence` | `float32` | Person identity confidence |
| `id` | `int32` | Sensor-assigned person ID (not persistent across resets) |
| `is_facing` | `bool` | Whether the face is oriented toward the sensor |

The sensor reports up to 4 faces per read; each is published as a separate message.

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select person_sensor_node
source install/setup.bash
```

---

## Run

```bash
ros2 run person_sensor_node person_sensor_node
```

---

## Notes

- If the sensor is not found on any I2C bus, the node starts anyway but logs an error and skips all timer callbacks.
- I2C reads that take >50ms are logged as warnings (usually indicates bus contention or a slow bus clock).
- The sensor ID (`id` field) is hardware-assigned and resets on power cycle; it's not a persistent identity. The `person_tracking_node` assigns stable internal IDs.
- `PERSON_SENSOR_FACE_MAX = 4` — the sensor supports a maximum of 4 simultaneous face detections.
