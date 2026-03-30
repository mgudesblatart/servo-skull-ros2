# servo_skull_launch

Orchestration package. No executable nodes — just launch files that wire the full system together with sequenced readiness checks.

---

## Launch Files

### `test_full_audio_pipeline.launch.py` — Full system

The main launch file for normal operation. Starts all audio/speech/LLM nodes in dependency order, gating each stage on the previous stage's `/ready` topic.

**Startup sequence:**
```
axllm_native_backend (start script + preflight) →
microphone_node → [wait ready] →
speaker_node    → [wait ready] →
tts_node        → [wait ready] →
skull_control_node + stt_node → [wait stt ready] →
[wait axllm /v1/models ready] →
llm_agent_http  (via skull_control_node/llm_agent_http.launch.py)
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

# Override backend URL consumed by llm_agent_http
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  axllm_base_url:=http://127.0.0.1:8081
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `mic_device` | `-1` | microphone_node device index (-1 = PortAudio default) |
| `speaker_device` | `""` | speaker_node device (int index or string; empty = auto-detect USB) |
| `axllm_startup_script` | `/home/murray/projects/servo-skull/scripts/start_axllm_native_serve.sh` | Native backend startup wrapper |
| `axllm_model_dir` | `/home/murray/models/Qwen3-1.7B` | Model directory for `axllm serve` |
| `axllm_port` | `8081` | Native backend port |
| `axllm_bin` | `/home/murray/projects/ax-llm/build_native/axllm` | Native `axllm` binary path |
| `axllm_config_path` | `""` | Optional `config.json` override for startup script |
| `axllm_base_url` | `http://127.0.0.1:8081` | URL used by `llm_agent_http_node` |
| `axllm_ready_timeout` | `60.0` | Seconds to wait for `/v1/models` readiness |

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

> **Note:** Backend readiness is checked explicitly via `GET <axllm_base_url>/v1/models` before `llm_agent_http` is started.

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
