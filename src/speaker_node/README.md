# speaker_node

Python ROS2 node. Receives `AudioData` messages and plays them through a persistent `sounddevice.OutputStream`. Handles resampling from the TTS output rate (22050 Hz) to playback rate (48000 Hz).

---

## Node: `speaker_node`

**Pipeline position:** TTS → **Speaker**

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/speaker/audio_output` | `servo_skull_msgs/AudioData` | int16 PCM audio chunks |
| Publish | `/speaker_node/ready` | `std_msgs/Bool` | Latched; `true` once OutputStream is open and playback thread is running |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `device` | `None` | Output device: integer index or device name string. Empty/None triggers auto-detection. |

---

## Device Selection

If `device` is not set, the node auto-detects in priority order:
1. First device with `usb` in the name and at least 1 output channel
2. First device with `speaker` or `output` in the name
3. `sounddevice` default output device

All detected devices are printed at startup. If the wrong device is selected, pass the index or a substring of the name via the `device` parameter.

```bash
# List devices at startup:
ros2 run speaker_node speaker_node 2>&1 | head -30

# Force a specific device:
ros2 run speaker_node speaker_node --ros-args -p device:=3
ros2 run speaker_node speaker_node --ros-args -p device:="USB Audio"
```

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select speaker_node
source install/setup.bash
```

---

## Run

```bash
ros2 run speaker_node speaker_node
```

---

## Notes

- The `OutputStream` is opened once at init at 48000 Hz, 1 channel, `float32`. Audio arriving at 22050 Hz (from Piper TTS) is resampled with `scipy.signal.resample_poly` before being written to the stream.
- Playback runs in a separate daemon thread (`playback_worker`). The audio queue holds up to 100 chunks.
- `JACK_NO_START_SERVER=1` is set by the full pipeline launch to suppress JACK audio daemon errors on systems without JACK. If running the node in isolation and you see JACK errors, `export JACK_NO_START_SERVER=1` before launching.
- The `is_last_chunk` flag in `AudioData` is unused at this layer; all chunks are played in order regardless.
