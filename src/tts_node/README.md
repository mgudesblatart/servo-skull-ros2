# tts_node

Python ROS2 node. Text-to-speech synthesis using [Piper](https://github.com/rhasspy/piper). Subscribes to text input, synthesises audio with a Piper voice model, and publishes chunked `AudioData` messages to the speaker.

---

## Node: `tts_node`

**Pipeline position:** LLM Agent → **TTS** → Speaker

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/text_to_speech/text_input` | `std_msgs/String` | Text to synthesise |
| Publish | `/speaker/audio_output` | `servo_skull_msgs/AudioData` | int16 PCM audio chunks at 22050 Hz |
| Publish | `/tts_node/ready` | `std_msgs/Bool` | Latched; `true` once Piper model is loaded |

---

## Model

**Default model:** `en_GB-alan-medium.onnx` (British male voice)

The model is bundled at `src/tts_node/models/` and installed to `share/tts_node/models/`.

Override model at runtime via the `PIPER_MODEL_PATH` environment variable (relative to the models directory):

```bash
export PIPER_MODEL_PATH=en_US-amy-medium.onnx
ros2 run tts_node tts_node
```

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select tts_node
source install/setup.bash
```

---

## Run

```bash
ros2 run tts_node tts_node
```

Test it directly (requires speaker_node to be running):
```bash
ros2 topic pub --once /text_to_speech/text_input std_msgs/String "data: 'Ave Dominus Deus'"
```

---

## Notes

- Synthesis runs in a background worker thread; the ROS subscriber callback just enqueues text. The queue holds up to 100 items; excess requests are dropped with an error log.
- Audio is published at 22050 Hz (Piper default). The `speaker_node` resamples to 48000 Hz for playback.
- `DEBUG_WRITE_WAV = False` in source — flip it to `True` during development to write synthesised audio to a WAV file for offline inspection.
- If the Piper model fails to load on startup, the node still starts and publishes `ready: false`. Synthesis requests are dropped with an error log. Check the model path and that `piper` is installed in the venv.
- `piper` must be installed in the Python venv: `pip install piper-tts` (or equivalent). It's not a ROS package.
