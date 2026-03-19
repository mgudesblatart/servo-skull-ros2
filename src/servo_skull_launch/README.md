# servo_skull_launch

Orchestration package. No executable nodes — just launch files that wire the full system together with sequenced readiness checks.

---

## Launch Files

### `test_full_audio_pipeline.launch.py` — Full system

The main launch file for normal operation. Starts all audio/speech/LLM nodes in dependency order, gating each stage on the previous stage's `/ready` topic.

**Startup sequence:**
```
microphone_node → [wait ready] →
speaker_node    → [wait ready] →
tts_node        → [wait ready] →
skull_control_node + stt_node → [wait stt ready] →
llm_agent_axcl  (via skull_control_node/llm_agent_axcl.launch.py)
```

`skull_control_node` is started before STT readiness completes so FSM/control topics are available early in the test pipeline.

Each readiness check waits up to its configured timeout (30s for mic/speaker, 45s for TTS/STT). If a stage fails to become ready, the entire launch shuts down.

**Basic usage:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
source ~/projects/servo-skull/install/setup.bash

ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py
```

**Common overrides:**
```bash
# Specify audio devices explicitly (use ros2 run microphone_node microphone_node --ros-args -p device_index:=? to enumerate)
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  mic_device:=2 \
  speaker_device:=3

# Override AXCL config
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  axcl_config_path:=/path/to/custom_config.yaml

# Start the tokenizer service as part of this launch (default: true)
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  start_tokenizer:=false
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `mic_device` | `-1` | microphone_node device index (-1 = PortAudio default) |
| `speaker_device` | `""` | speaker_node device (int index or string; empty = auto-detect USB) |
| `axcl_config_path` | package share `axcl_servo_skull.yaml` | AXCL agent config YAML |
| `runtime_command` | `""` | Override AXCL runtime shell command |
| `runtime_cwd` | `""` | Override AXCL runtime working directory |
| `tokenizer_script` | `~/models/Qwen2.5-1.5B-Instruct/qwen2.5_tokenizer_uid.py` | Tokenizer service script |
| `tokenizer_cwd` | `~/models/Qwen2.5-1.5B-Instruct` | Tokenizer service working directory |
| `start_tokenizer` | `true` | Whether to start the tokenizer service |
| `inline_system_prompt` | `""` | Override `inline_system_prompt` in AXCL config (empty = use config file value) |

---

### `test_audio_stt.launch.py` — Audio + STT only

Microphone and STT only. Useful for testing transcription without the LLM or TTS in the loop.

```bash
ros2 launch servo_skull_launch test_audio_stt.launch.py
```

---

### `test_tts_speaker.launch.py` — TTS + Speaker only

Starts speaker and TTS nodes, then publishes a test phrase to `/text_to_speech/text_input`. Useful for verifying audio output and Piper voice without the full pipeline.

```bash
ros2 launch servo_skull_launch test_tts_speaker.launch.py
```

---

### `testing.launch.py` — Servo/tracking test

Starts person sensor, person tracking, ESP32 interface, and skull control (BT servo node). Useful for testing servo pan/tilt and face tracking without the audio pipeline.

```bash
ros2 launch servo_skull_launch testing.launch.py
```

---

## Readiness Protocol

All Python-based nodes publish a latched `std_msgs/Bool` to `/<node_name>/ready` when initialised. The full pipeline launch uses an inline Python waiter script (`WAIT_FOR_READY_SCRIPT`) that subscribes with `TRANSIENT_LOCAL` durability, so late-joining waiters still get the last published value.

Readiness topics:
- `/microphone_node/ready`
- `/speaker_node/ready`
- `/tts_node/ready`
- `/stt_node/ready`

> **Note:** `llm_agent_axcl_node` does **not** currently publish a ready topic. The AXCL launch is started after STT is ready; the runtime startup timeout is handled internally by `AxclRuntimeClient`.

---

## Test Hook Behavior

`skull_control_node` exposes an optional test-only topic `/skull_control/test_event` for deterministic FSM forcing used by smoke scripts.

- In this full test pipeline launch, `skull_control_node` is started with `enable_test_events: true`.
- In non-test launches, `enable_test_events` remains disabled (default `false`, and explicitly set to `false` in testing launch files).

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select servo_skull_launch
source install/setup.bash
```

---

## Notes

- `JACK_NO_START_SERVER=1` is set in the full pipeline launch to prevent spurious JACK audio server errors on systems without JACK installed.
- `PYTHONPATH` is extended to include the venv site-packages at launch time. This is necessary because ROS2 launch processes don't inherit the activated venv.
- All nodes write their output to `screen` so you can see them in the terminal. Redirect with `output:=log` if you prefer quieter launches.
