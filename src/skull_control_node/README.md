# skull_control_node

Core skull behaviour package. Contains the HTTP-backed LLM agent and the BT-based servo skull controller.

---

## Nodes

### `llm_agent_http_node`

Listens for typed `LlmInput` envelopes, calls an OpenAI-compatible `axllm serve` backend over HTTP, parses the JSON contract response, and publishes TTS phrases.

Guardrails in current implementation:
- Runtime/log contamination is filtered before speech publish.
- Strict response contract uses only `thoughts` + `tool_calls`.
- For `channel=human` inputs, if no `say_phrase` is produced, a sanitized fallback phrase is spoken.

**Pipeline position:** STT → skull_control gate (`/skull_control/llm_input`) → **LLM Agent** → TTS

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/skull_control/llm_input` | `servo_skull_msgs/LlmInput` | Gated typed envelope from `skull_control_node` |
| Subscribe | `/llm_agent/control` | `std_msgs/String` | `CANCEL` / `HALT` / `STOP` interrupt control |
| Publish | `/text_to_speech/text_input` | `std_msgs/String` | Phrase to speak aloud |
| Publish | `/llm_agent/status` | `servo_skull_msgs/LlmStatus` | Runtime/health events for FSM recovery |

**Parameters** (set via config YAML; see below):

| Parameter | Default | Description |
|---|---|---|
| `config_path` | `""` | Path to HTTP config YAML |
| `axllm_base_url` | `"http://127.0.0.1:8081"` | Base URL for `axllm serve` |
| `axllm_model` | `"AXERA-TECH/Qwen3-1.7B"` | Model ID passed to `/v1/chat/completions` |
| `startup_timeout_sec` | `30.0` | Startup readiness timeout |
| `request_timeout_sec` | `60.0` | Per-request timeout |
| `max_history_turns` | `8` | Rolling conversation history size |
| `system_prompt` | `""` | Optional runtime override of config prompt |

---

### `skull_control_node` (`skull_control_bt_node`)

BT-based servo controller. Subscribes to tracked person data, selects a target using a Behaviour Tree, and publishes pan/tilt commands for the ESP32. Tracks are maintained with a timeout; the skull stops tracking if no person is seen for 5 seconds.

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/person_tracking/tracked_persons` | `servo_skull_msgs/TrackedPersons` | Face track data |
| Subscribe | `/speech_to_text/transcript` | `std_msgs/String` | Raw STT input for FSM gating/interrupt detection |
| Subscribe | `/text_to_speech/text_input` | `std_msgs/String` | Verbal-response start hook for FSM transitions |
| Subscribe | `/speaker/audio_output` | `servo_skull_msgs/AudioData` | Uses `is_last_chunk` for `tts_done` transition |
| Subscribe | `/speaker_node/playback_timing` | `servo_skull_msgs/PlaybackTiming` | Dynamic echo-suppression timing from speaker |
| Subscribe | `/llm_agent/status` | `servo_skull_msgs/LlmStatus` | LLM runtime health/status events |
| Subscribe (optional) | `/skull_control/test_event` | `std_msgs/String` | Test-only FSM forcing hook (disabled by default) |
| Publish | `skull_control/pan_tilt_cmd` | `geometry_msgs/Point` | Pan (x) / tilt (y) in [0–500] range |
| Publish | `/skull_control/state_transition` | `servo_skull_msgs/StateTransition` | Typed transition trace (`from_state`, `to_state`, `event`, `ts`) |
| Publish | `/skull_control/llm_input` | `servo_skull_msgs/LlmInput` | Typed event-aware envelope stream for LLM agent |
| Publish | `/tts_node/control` | `std_msgs/String` | Interrupt control (`STOP`) |
| Publish | `/speaker_node/control` | `std_msgs/String` | Interrupt control (`STOP`) |
| Publish | `/llm_agent/control` | `std_msgs/String` | Interrupt control (`CANCEL`) |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `enable_test_events` | `false` | Enables `/skull_control/test_event` subscription for deterministic smoke tests |

---

## Configuration

Primary config file: `configs/http_servo_skull.yaml`

```yaml
axllm_base_url: "http://127.0.0.1:8081"
axllm_model: "AXERA-TECH/Qwen3-1.7B"
startup_timeout_sec: 30.0
request_timeout_sec: 90.0
max_history_turns: 8
system_prompt: |-
  /no_think
  You are a servitor unit designated Servo Skull.
  ...
```

---

## JSON Contract

The LLM is instructed to respond with exactly one JSON object per inference:

```json
{
  "thoughts": "Reasoning/planning string (not spoken)",
  "tool_calls": [
    { "say_phrase": { "msg": "Phrase to synthesise and speak aloud." } }
  ]
}
```

`response_parser.py` handles the following fallback chain:
1. Strict JSON parse after stripping markdown fences
2. Balanced brace extraction + parse
3. Regex `{.*}` extraction + parse
4. `json_repair` (if installed) applied to all candidates
5. Plain text fallback — wraps raw output in a synthetic contract so the skull still speaks something

---

## LLM Input Envelope

`skull_control_bt_node` publishes typed `servo_skull_msgs/LlmInput` on `/skull_control/llm_input`.

Fields:
- `channel`: `human | system | mixed`
- `source`: `stt | person_tracking | fsm | ...`
- `type`: `transcript | event`
- `event`: `human_transcript | person_detected | ...`
- `urgency`: `low | medium | high`
- `text`: optional natural-language text
- `payload_json`: optional JSON metadata object encoded as string
- `ts`: `time.time()` publish timestamp

Current producers:
- Human speech: `channel=human`, `type=transcript`, `source=stt`, `urgency=medium`.
- System events: `channel=system`, `type=event` for:
  - `person_detected`
  - `no_motion_timeout`
  - `no_speech_timeout`
  - `low_interest_timeout`
  - `interrupt_detected`
  - `tts_done`

Anti-spam behavior in BT node:
- Per-event cooldown (5-12s depending on event).
- Additional suppression for identical `event + payload` repeats.

Speech-loop suppression in BT node:
- Short post-TTS blanket suppression window for STT (`POST_TTS_ECHO_SUPPRESS_SEC`).
- Content-aware echo filtering: drops STT transcripts that closely match the most recent TTS phrase.
- Interrupt tokens (`HALT` / `HOLD`) bypass suppression.

---

## HTTP Runtime

`llm_agent_http_node` talks to `axllm serve` over OpenAI-compatible HTTP (`/v1/chat/completions`).

Backend readiness is checked via `/v1/models` before prompt traffic is accepted, and runtime failures are published on `/llm_agent/status` using typed `servo_skull_msgs/LlmStatus`.

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select skull_control_node
source install/setup.bash
```

---

## Launch

### LLM Agent only

```bash
ros2 launch skull_control_node llm_agent_http.launch.py
```

Override config:
```bash
ros2 launch skull_control_node llm_agent_http.launch.py \
  config_path:=/path/to/custom.yaml
```

### Skull control + person tracking + ESP32 test launch

```bash
ros2 launch skull_control_node testing.launch.py
```

This starts: `person_sensor_node`, `person_tracking_node`, `servo_skull_esp32`, and `skull_control_bt_node`.

### Full pipeline (recommended — use `servo_skull_launch`)

See [`servo_skull_launch`](../servo_skull_launch/README.md) for the full orchestrated pipeline launch.

---

## Key Implementation Files

| File | Purpose |
|---|---|
| `llm_agent_http_node.py` | Main HTTP-backed ROS2 LLM agent |
| `axllm_http_client.py` | OpenAI-compatible HTTP client wrapper for `axllm serve` |
| `response_parser.py` | JSON contract parser with multi-strategy fallback chain |
| `skull_control_bt_node.py` | BT-based servo pan/tilt tracking node |
| `skull_control_node.py` | Core skull control logic |
| `configs/http_servo_skull.yaml` | Active HTTP agent configuration |

---

## Notes

- `llm_agent_http_node` uses a `MultiThreadedExecutor` + `ReentrantCallbackGroup` so control callbacks (`/llm_agent/control`) can cancel active requests while prompt handling is running.
- `llm_agent_http_node` and `skull_control_bt_node` now exchange typed `servo_skull_msgs` envelopes/status for `/skull_control/llm_input`, `/llm_agent/status`, and `/skull_control/state_transition`.
- Runtime startup and serving are handled by `axllm serve` (via launch orchestration), not an AXCL subprocess inside the node.
