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
source ../venv-servo-skull/bin/activate &&
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

