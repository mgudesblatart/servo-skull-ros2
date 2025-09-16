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
```
```shell
ps aux | grep ros2
kill <PID>
```
```shell
rosdep install --from-paths src --ignore-src -r -y
```
