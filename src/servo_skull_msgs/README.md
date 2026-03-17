# servo_skull_msgs

Custom ROS2 message definitions shared across the servo skull packages. No executables — just message types.

---

## Messages

### `AudioData`

Raw audio payload for inter-node audio transport (TTS → Speaker).

| Field | Type | Description |
|---|---|---|
| `data` | `int16[]` | Raw PCM samples (16-bit signed, little-endian) |
| `sample_rate` | `uint32` | Sample rate in Hz (typically 22050 from Piper) |
| `channels` | `uint8` | Channel count (1 = mono) |
| `is_last_chunk` | `bool` | Marks the final chunk of an utterance (informational; not currently acted on by speaker_node) |

---

### `PersonDetection`

Single face detection from the Useful Sensors Person Sensor (raw, untracked).

| Field | Type | Description |
|---|---|---|
| `box_confidence` | `float32` | Detection confidence [0.0–1.0] |
| `box_left` | `float32` | Left bounding box edge [0–255] |
| `box_top` | `float32` | Top bounding box edge [0–255] |
| `box_right` | `float32` | Right bounding box edge [0–255] |
| `box_bottom` | `float32` | Bottom bounding box edge [0–255] |
| `id_confidence` | `float32` | Person identity confidence |
| `id` | `int32` | Sensor-assigned person ID |
| `is_facing` | `bool` | True if the face is oriented toward the sensor |

---

### `TrackedPerson`

A single tracked person with a stable internal ID, assigned and maintained by `person_tracking_node`.

| Field | Type | Description |
|---|---|---|
| `person_id` | `int32` | Stable internal track ID (assigned by tracking node; persists across sensor ID changes) |
| `name` | `string` | Always `"unknown"` (identity recognition not implemented) |
| `box_left` | `float32` | Last known left bounding box edge |
| `box_top` | `float32` | Last known top bounding box edge |
| `box_right` | `float32` | Last known right bounding box edge |
| `box_bottom` | `float32` | Last known bottom bounding box edge |
| `box_confidence` | `float32` | Detection confidence of last update |
| `sensor_id` | `int32` | Original sensor-assigned ID |
| `age` | `int32` | Total number of detections for this track |
| `visible_count` | `int32` | Number of times this track was visible |
| `state` | `string` | Track state: `"active"`, `"lost"`, or `"deleted"` |

---

### `TrackedPersons`

Container for all current tracked persons.

| Field | Type | Description |
|---|---|---|
| `tracks` | `TrackedPerson[]` | Array of all active/lost tracked persons |

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
colcon build --packages-select servo_skull_msgs
source install/setup.bash
```

This package must be built before any package that depends on these message types (`speaker_node`, `stt_node`, `tts_node`, `person_sensor_node`, `person_tracking_node`, `skull_control_node`).
