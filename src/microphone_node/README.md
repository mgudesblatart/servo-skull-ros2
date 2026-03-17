# microphone_node

C++ ROS2 node. Captures audio from a PortAudio input device and publishes raw PCM frames as `UInt8MultiArray`.

---

## Node: `microphone_node`

**Pipeline position:** **Microphone** → STT

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Publish | `audio/raw` | `std_msgs/UInt8MultiArray` | Raw 16-bit LE PCM audio frames |
| Publish | `/microphone_node/ready` | `std_msgs/Bool` | Latched; `true` once PortAudio stream is open |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `sample_rate` | `48000` | Capture sample rate in Hz |
| `channels` | `1` | Number of input channels |
| `frames_per_buffer` | `9600` | PortAudio frames per callback (≈ 200ms at 48kHz) |
| `bit_depth` | `16` | Bit depth (only 16 is tested/used downstream) |
| `device_index` | `-1` | PortAudio device index; -1 uses system default input |

---

## Build

```bash
sudo apt-get install portaudio19-dev libasound2-dev

cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
colcon build --packages-select microphone_node
source install/setup.bash
```

---

## Run

```bash
ros2 run microphone_node microphone_node
```

Override device:
```bash
ros2 run microphone_node microphone_node --ros-args -p device_index:=2
```

List available PortAudio input devices (printed to stdout at node startup):
```bash
ros2 run microphone_node microphone_node 2>&1 | head -20
```

---

## Notes

- ALSA error messages during PortAudio initialisation are silenced intentionally (it probes all devices). Actual stream open/start failures are still logged via `RCLCPP_ERROR`.
- Audio is published as raw `uint8` bytes (little-endian 16-bit signed PCM). The `stt_node` deserialises these back to `int16` → `float32` internally.
- Queue size is capped at 50 frames. If the subscriber (STT) can't keep up, frames are dropped on the publisher side.
- USB microphones are typically auto-selected when `device_index=-1` only if they're the system default. Use explicit `device_index` if you have multiple input devices and want deterministic selection.
