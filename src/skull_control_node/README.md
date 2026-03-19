# skull_control_node

Core skull behaviour package. Contains two independent nodes: the AXCL LLM agent and the BT-based servo skull controller.

---

## Nodes

### `llm_agent_axcl_node`

Listens for speech transcripts, feeds them to an AXCL-hosted LLM via a persistent subprocess, parses the JSON contract response, and publishes TTS phrases. This is the "brain" of the servo skull.

**Pipeline position:** STT → skull_control gate (`/skull_control/llm_input`) → **LLM Agent** → TTS

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/skull_control/llm_input` | `std_msgs/String` | Gated transcript input forwarded by `skull_control_node` |
| Subscribe | `/llm_agent_axcl/control` | `std_msgs/String` | `CANCEL` / `HALT` / `STOP` interrupt control |
| Publish | `/text_to_speech/text_input` | `std_msgs/String` | Phrase to speak aloud |

**Parameters** (set via config YAML; see below):

| Parameter | Default | Description |
|---|---|---|
| `config_path` | `""` | Path to AXCL config YAML (auto-resolved to package share path in launch) |
| `runtime_command` | `""` | Shell command to start the AXCL interactive model runtime |
| `runtime_cwd` | `""` | Working directory for the runtime process |
| `prompt_marker` | `"prompt >>"` | String the runtime prints when ready for input |
| `startup_timeout_sec` | `120.0` | Max seconds to wait for runtime startup handshake |
| `response_timeout_sec` | `60.0` | Max seconds to wait for a single inference response |
| `inline_system_prompt` | `""` | Override for inline_system_prompt config flag (true/false/empty) |

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
| Subscribe (optional) | `/skull_control/test_event` | `std_msgs/String` | Test-only FSM forcing hook (disabled by default) |
| Publish | `skull_control/pan_tilt_cmd` | `geometry_msgs/Point` | Pan (x) / tilt (y) in [0–500] range |
| Publish | `/skull_control/state_transition` | `std_msgs/String` | Transition trace JSON (`from`, `to`, `event`, `ts`) |
| Publish | `/skull_control/llm_input` | `std_msgs/String` | Gated transcript stream for LLM agent |
| Publish | `/tts_node/control` | `std_msgs/String` | Interrupt control (`STOP`) |
| Publish | `/speaker_node/control` | `std_msgs/String` | Interrupt control (`STOP`) |
| Publish | `/llm_agent_axcl/control` | `std_msgs/String` | Interrupt control (`CANCEL`) |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `enable_test_events` | `false` | Enables `/skull_control/test_event` subscription for deterministic smoke tests |

---

## Configuration

Primary config file: `configs/axcl_servo_skull.yaml`

```yaml
runtime_command: "/home/murray/projects/servo-skull/scripts/run_qwen2_5_axcl_custom.sh"
runtime_cwd: "/home/murray/projects/servo-skull"
prompt_marker: "prompt >>"
startup_timeout_sec: 120.0
response_timeout_sec: 60.0

# false = system_prompt injected once at runtime startup via SYSTEM_PROMPT env var (recommended)
# true  = prepended to every user message (expensive; causes context pollution)
inline_system_prompt: false

system_prompt: |-
  You are a servitor unit designated Servo Skull.
  ...
```

**`inline_system_prompt` guidance:** Keep this `false`. When `false`, the node sets `SYSTEM_PROMPT` in the runtime process environment before startup — the model gets the system prompt once. When `true`, the system prompt is prepended to every user message as a single flattened line, which balloons context length and degrades coherence quickly.

---

## JSON Contract

The LLM is instructed to respond with exactly one JSON object per inference:

```json
{
  "thoughts": "Reasoning/planning string (not spoken)",
  "tool_calls": [
    { "say_phrase": { "msg": "Phrase to synthesise and speak aloud." } }
  ],
  "final_output": "Phrase said."
}
```

`response_parser.py` handles the following fallback chain:
1. Strict JSON parse after stripping markdown fences
2. Balanced brace extraction + parse
3. Regex `{.*}` extraction + parse
4. `json_repair` (if installed) applied to all candidates
5. Plain text fallback — wraps raw output in a synthetic contract so the skull still speaks something

---

## AXCL Runtime

The node spawns `run_qwen2_5_axcl_custom.sh` as a persistent subprocess (via `axcl_runtime_client.py`). The runtime is interactive: the node writes a prompt line to stdin and reads stdout until the `prompt >>` marker reappears.

The startup handshake waits up to `startup_timeout_sec` for the first `prompt >>` — model loading on the Orbbec/AXCL device takes ~60s on first run.

All stdout/stderr from the runtime is merged (`stderr=STDOUT`) and filtered by the node before logging; lines matching `[X][...` (runtime log lines) are stripped.

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
ros2 launch skull_control_node llm_agent_axcl.launch.py
```

Override config:
```bash
ros2 launch skull_control_node llm_agent_axcl.launch.py \
  config_path:=/path/to/custom.yaml \
  inline_system_prompt:=false
```

Start the tokenizer service alongside the agent (required if not already running):
```bash
ros2 launch skull_control_node llm_agent_axcl.launch.py start_tokenizer:=true
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
| `llm_agent_axcl_node.py` | Main ROS2 node; subscription/publish logic; prompt construction |
| `axcl_runtime_client.py` | Persistent subprocess wrapper; thread-safe stdin/stdout I/O |
| `response_parser.py` | JSON contract parser with multi-strategy fallback chain |
| `agent_tools.py` | Tool definitions for the agent |
| `skull_control_bt_node.py` | BT-based servo pan/tilt tracking node |
| `skull_control_node.py` | Core skull control logic |
| `configs/axcl_servo_skull.yaml` | Active AXCL agent configuration |

---

## Notes

- `llm_agent_axcl_node` uses a `MultiThreadedExecutor` + `ReentrantCallbackGroup` so control callbacks (`/llm_agent_axcl/control`) can cancel active requests while prompt handling is running.
- `axcl_runtime_client.py` uses a `threading.Lock` on `generate()` calls; only one prompt can be in-flight at a time.
- If the runtime process dies mid-session, `generate()` will raise a `RuntimeError`. The node logs the error and continues listening; a restart is required to re-establish the subprocess.
- The tokenizer service (`qwen2.5_tokenizer_uid.py`) must be running before the AXCL runtime starts. The launch file can optionally start it via `start_tokenizer:=true`.
