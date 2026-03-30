```shell
ros2 pkg create --build-type ament_python my_package_name
```
```shell
ros2 pkg create --build-type ament_cmake my_package_name
```
```shell
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py mic_device:=0 speaker_device:=1
ros2 launch skull_control_node skull_control.launch.py
ros2 launch skull_control_node test_llama.launch.py prompt:="good evening"
```
```shell
ros2 run skull_control_node llm_agent_http_node --ros-args -p config_path:=/home/murray/projects/servo-skull/src/skull_control_node/configs/http_servo_skull.yaml
```
```shell
ps aux | grep ros2
kill <PID>
```
```shell
rosdep install --from-paths src --ignore-src -r -y
```
# set up env
```shell
source /opt/ros/jazzy/setup.bash &&
source ~/projects/venv-servo-skull/bin/activate &&
source ./install/setup.bash
```

```shell
# rebuild after code/launch changes
source /opt/ros/jazzy/setup.bash &&
source ~/projects/venv-servo-skull/bin/activate &&
colcon build --packages-select skull_control_node servo_skull_launch
source ./install/setup.bash
```

```shell
# HTTP agent launch (default)
source /opt/ros/jazzy/setup.bash &&
source ~/projects/venv-servo-skull/bin/activate &&
source ./install/setup.bash &&
ros2 launch skull_control_node skull_control.launch.py

# HTTP agent launch with explicit config override
ros2 launch skull_control_node llm_agent_http.launch.py \
  config_path:=/home/murray/projects/servo-skull/src/skull_control_node/configs/http_servo_skull.yaml
```

```shell
# AXLLM native backend preflight (guards against HTTP tokenizer URL mismatch)
bash /home/murray/projects/servo-skull/scripts/axllm_native_tokenizer_preflight.sh

# AXLLM native backend startup helper (runs preflight, then starts axllm serve)
bash /home/murray/projects/servo-skull/scripts/start_axllm_native_serve.sh

# AXLLM native backend launch (recommended for OpenAI-compatible /v1 API)
source /opt/ros/jazzy/setup.bash &&
source ~/projects/venv-servo-skull/bin/activate &&
source ./install/setup.bash &&
ros2 launch servo_skull_launch axllm_native_backend.launch.py

# Optional overrides
ros2 launch servo_skull_launch axllm_native_backend.launch.py \
  model_dir:=/home/murray/models/Qwen3-1.7B \
  port:=8081 \
  axllm_bin:=/home/murray/projects/ax-llm/build_native/axllm
```

```shell
# Startup helper supports flags (no positional args)
bash /home/murray/projects/servo-skull/scripts/start_axllm_native_serve.sh \
  --model-dir /home/murray/models/Qwen3-1.7B \
  --port 8081 \
  --axllm-bin /home/murray/projects/ax-llm/build_native/axllm
```

## LLM Backend Selection

The skull defaults to the HTTP-backed agent `llm_agent_http_node`.
- **Config File**: `src/skull_control_node/configs/http_servo_skull.yaml`
- **Server API**: OpenAI-compatible `/v1/chat/completions` (from `axllm serve`)
- **Recommended launch**: run the native backend and the skull control launch together.

## Integration Test: STT → skull_control gate → LLM → TTS

```shell
# Terminal 1: Launch full test pipeline
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py

# Terminal 2: Monitor gated LLM input
ros2 topic echo /skull_control/llm_input

# Terminal 3: Monitor TTS output
ros2 topic echo /text_to_speech/text_input

# Terminal 4: Publish test transcript
ros2 topic pub --once /speech_to_text/transcript std_msgs/String "data: 'Good evening'"
```

Expect gated transcript forwarding on `/skull_control/llm_input`, then LLM-driven phrase output on `/text_to_speech/text_input`.

## FSM / Interrupt Smoke Tests

```shell
# quick interrupt-only check (HALT -> STOP/STOP/CANCEL + interrupt transition)
./scripts/pi_interrupt_quickcheck.sh

# full gate + interrupt check
./scripts/pi_fsm_gate_smoketest.sh
```

Notes:
- These scripts expect the full test pipeline launch to be running.
- The test hook topic `/skull_control/test_event` is enabled only by the full test pipeline launch.
- Non-test launches keep `enable_test_events` disabled.

## Testing Tips

- **Graceful error handling**: Publish garbled input (e.g. invalid JSON) to verify the response_parser fallback chain works and no crash occurs
- **Latency measurement**: Compare timestamp from STT publish → `/skull_control/llm_input` → `/text_to_speech/text_input` to establish baseline performance
- **Full pipeline**: `ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py` (if audio hardware available)
-

## Evaluate AXLLM Model Usage:

Run against another endpoint:
/home/murray/projects/venv-servo-skull/bin/python /home/murray/projects/servo-skull/scripts/evaluate_axllm_server.py
Run only one or two cases:
```shell
/home/murray/projects/venv-servo-skull/bin/python /home/murray/projects/servo-skull/scripts/evaluate_axllm_server.py \
  --base-url http://127.0.0.1:8082 \
  --model AXERA-TECH/Qwen3-1.7B
```
Use a different prompt file:
```shell
/home/murray/projects/venv-servo-skull/bin/python /home/murray/projects/servo-skull/scripts/evaluate_axllm_server.py \
  --system-prompt-file /path/to/prompt.txt
```
Run a subset of cases:
```shell
/home/murray/projects/venv-servo-skull/bin/python /home/murray/projects/servo-skull/scripts/evaluate_axllm_server.py \
  --only greeting,personality
```
Machine-readable output:
```shell
/home/murray/projects/venv-servo-skull/bin/python /home/murray/projects/servo-skull/scripts/evaluate_axllm_server.py \
  --json
```