# person_tracking_node

C++ ROS2 node. Consumes raw `PersonDetection` messages from the Useful Sensors person sensor and maintains a map of tracked persons with stable internal IDs across frames.

---

## Node: `person_tracking_node`

**Pipeline position:** Person Sensor → **Person Tracking** → Skull Control

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/person_sensor/detections` | `servo_skull_msgs/PersonDetection` | Raw per-frame face detections |
| Publish | `/person_tracking/tracked_persons` | `servo_skull_msgs/TrackedPersons` | Active tracks with stable IDs |

---

## Track Lifecycle

- **New detection (id not seen before):** Creates a new track with an auto-incrementing `internal_id`.
- **Known detection (same sensor id):** Updates position, confidence, age, and `visible_count`.
- **No detection for a track:** Track enters `lost` state. After a configurable stale timeout (1s cleanup timer), `deleted` tracks are pruned.
- **Low confidence detections** (`box_confidence < 0.5`) are discarded before tracking.

**Track states:** `active`, `lost`, `deleted`

`TrackedPerson.person_id` is the stable `internal_id` assigned by this node — not the raw sensor ID. The skull control BT node uses `person_id` for target continuity.

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
colcon build --packages-select person_tracking_node
source install/setup.bash
```

---

## Run

```bash
ros2 run person_tracking_node person_tracking_node
```

---

## Notes

- The cleanup timer fires every 1 second and removes stale/deleted tracks from the internal map.
- 3D position/velocity fields in `TrackedPerson` are currently unused (TODO markers in source); the sensor only provides 2D bounding boxes.
- The `name` field is always `"unknown"` — no identity recognition is implemented.