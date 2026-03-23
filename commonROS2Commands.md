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
ros2 run skull_control_node llm_agent_node
ros2 run skull_control_node llm_agent_axcl_node --ros-args -p config_path:=/mnt/d/Projects/servo-skull-rpi/src/skull_control_node/configs/axcl_servo_skull.yaml
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
# custom AXCL runtime launcher (editable script)
/home/murray/projects/servo-skull/scripts/run_qwen2_5_axcl_custom.sh

# AXCL node launch (recommended): one-time startup system prompt via runtime script
# (uses axcl_servo_skull.yaml system_prompt when inline_system_prompt: false)
# NOTE: If SYSTEM_PROMPT is already exported in your shell, it overrides YAML.

source /opt/ros/jazzy/setup.bash &&
source ~/projects/venv-servo-skull/bin/activate &&
source ./install/setup.bash &&
ros2 launch skull_control_node llm_agent_axcl.launch.py start_tokenizer:=false

# Optional debug/override only: force per-turn prompt prepend
ros2 launch skull_control_node llm_agent_axcl.launch.py start_tokenizer:=true inline_system_prompt:=true
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
  model_dir:=/home/murray/models/Qwen2.5-1.5B-Instruct \
  port:=8011 \
  axllm_bin:=/home/murray/projects/ax-llm/build_native/axllm
```

```shell
# One-time tokenizer export for native axllm (if tokenizer.txt missing)
cd /home/murray/projects/ax-llm/third_party/tokenizer.axera/tests &&
python3 convert_tokenizer.py \
  --tokenizer_path /home/murray/models/Qwen2.5-1.5B-Instruct/qwen2.5_tokenizer \
  --dst_path /home/murray/models/Qwen2.5-1.5B-Instruct/tokenizer.txt

# Then set config url_tokenizer_model to tokenizer.txt (not http://...)
```

## LLM Backend Selection

The skull defaults to **AXCL** (AX650 accelerator) for inference via `llm_agent_axcl_node`.
- **Config File**: `src/skull_control_node/configs/axcl_servo_skull.yaml`
- **Model Format**: `.axmodel` (AXERA proprietary, aarch64-compatible)
- **Model Location**: Place `.axmodel` files in `~/models/` or project-relative path and pass via launch arg:
  ```shell
  ros2 launch skull_control_node llm_agent_axcl.launch.py \
    runtime_command:=/path/to/main_axcl_aarch64 \
    runtime_cwd:=/path/to/model/dir
  ```

Alternative backends (e.g. llama.ros with CPU/GPU) can be swapped by editing `src/skull_control_node/launch/skull_control.launch.py` to load a different agent node.

## Integration Test: STT → skull_control gate → LLM → TTS

```shell
# Terminal 1: Launch full test pipeline
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py mic_device:=0 speaker_device:=1

# Optional: same pipeline + native AXLLM serve (preflight + serve auto-start)
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  mic_device:=0 speaker_device:=1 \
  start_axllm_native_backend:=true

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

# notes/scratchpad

        // "ANTHROPIC_BASE_URL": "http://192.168.68.68:11434",
        "ANTHROPIC_BASE_URL": "https://api.anthropic.com",
        "ANTHROPIC_API_KEY": "sk-no-need"
        // "CLAUDE_CODE_MAX_OUTPUT_TOKENS": "8192",
        // "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1",
        // "CLAUDE_CODE_ATTRIBUTION_HEADER": "0"