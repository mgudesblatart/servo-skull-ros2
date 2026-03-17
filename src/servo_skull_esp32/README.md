# servo_skull_esp32

Python ROS2 node. Serial interface to the ESP32 microcontroller. Relays servo pan/tilt commands and LED matrix commands over UART, and republishes serial messages from the ESP32 as ROS topics.

---

## Node: `interface_node` (executable: `servo_skull_esp32`)

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `skull_control/pan_tilt_cmd` | `geometry_msgs/Point` | x=pan, y=tilt in [0–500] range |
| Subscribe | `led_matrix/display_cmd` | `std_msgs/String` | Display text/pattern string |
| Publish | `esp32/ultrasonic` | `std_msgs/String` | Ultrasonic sensor readings from ESP32 |
| Publish | `esp32/info` | `std_msgs/String` | INFO messages from ESP32 serial |
| Publish | `esp32/error` | `std_msgs/String` | ERROR messages from ESP32 serial |

---

## Serial Protocol

**Baud rate:** 115200
**Port:** Auto-detected as first `/dev/ttyUSB*` device.

**Outgoing (Pi → ESP32):**
- Pan/tilt: `{x},{y},{checksum}\n` where checksum = `(x + y) & 0xFF`
- LED display: `DISPLAY:{command}\n`

**Incoming (ESP32 → Pi):**
- `ULTRASONIC:{value}` → published to `esp32/ultrasonic`
- `INFO:{message}` → published to `esp32/info`
- `ERROR:{message}` → published to `esp32/error`
- Unknown prefixes are logged as warnings

---

## Build

```bash
pip install pyserial  # if not already in venv

cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select servo_skull_esp32
source install/setup.bash
```

---

## Run

```bash
ros2 run servo_skull_esp32 servo_skull_esp32
```

---

## Notes

- Serial port is determined at import time by `find_serial_port()` (scans `/dev/ttyUSB*`). If no device is found at startup, the node raises `RuntimeError` and won't start. Ensure the ESP32 is plugged in before launching.
- DTR and RTS are explicitly set to `False` to prevent the ESP32 from auto-resetting on serial connect.
- A `threading.Lock` guards all serial writes; the read loop runs in a separate daemon thread.
- `test_led_display_cmd.py` in the package directory is a standalone script for testing LED matrix commands without ROS — run it directly with `python3 test_led_display_cmd.py`.
- User must have serial port permissions: `sudo usermod -aG dialout $USER` (requires re-login).
   Or to build a specific package:
   ```bash
   colcon build --packages-select servo_skull_esp32
   ```

4. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```

### Python Node
```bash
ros2 run servo_skull_esp32 interface_node
```
#### Debugging
```bash
ros2 run servo_skull_esp32 interface_node --ros-args --log-level interface_node:=debug
```

## Notes
- Make sure you have installed ROS 2 Humble (or later) for ARM64.
- Both nodes print a message every second to demonstrate functionality.
