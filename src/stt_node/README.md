# stt_node

Python ROS2 node. Streaming speech-to-text using Sherpa-ONNX with the Zipformer transducer model. Subscribes to raw audio, detects utterance endpoints, and publishes transcribed text.

---

## Node: `stt_node`

**Pipeline position:** Microphone → **STT** → LLM Agent

**ROS interface:**

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `audio/raw` | `std_msgs/UInt8MultiArray` | Raw 16-bit LE PCM at 48000 Hz |
| Publish | `/speech_to_text/transcript` | `std_msgs/String` | Transcribed utterance text |
| Publish | `/stt_node/ready` | `std_msgs/Bool` | Latched; `true` once Sherpa-ONNX recogniser is loaded |

---

## Model

**Model:** `sherpa-onnx-streaming-zipformer-en-2023-06-21`

Files bundled at `src/stt_node/models/` and installed to `share/stt_node/models/`:
- `encoder-epoch-99-avg-1.int8.onnx`
- `decoder-epoch-99-avg-1.onnx`
- `joiner-epoch-99-avg-1.int8.onnx`
- `tokens.txt`

The node resolves model paths via `ament_index_python.get_package_share_directory("stt_node")` — no manual path configuration needed.

---

## Recogniser Configuration

The Sherpa-ONNX `OnlineRecognizer` is configured with:

| Setting | Value |
|---|---|
| Provider | `cpu` |
| Threads | 2 |
| Input sample rate | 16000 Hz |
| Feature dim | 80 |
| Decoding method | `modified_beam_search` |
| Max active paths | 6 |
| Endpoint rule 1 (trailing silence) | 2.4s |
| Endpoint rule 2 (trailing silence) | 1.8s |
| Endpoint rule 3 (min utterance length) | 300 frames |

Incoming audio (48000 Hz) is resampled to 16000 Hz via `scipy.signal.resample_poly` before being fed to the recogniser.

---

## Build

```bash
cd ~/projects/servo-skull
source /opt/ros/jazzy/setup.bash
source ~/projects/venv-servo-skull/bin/activate
colcon build --packages-select stt_node
source install/setup.bash
```

---

## Run

```bash
ros2 run stt_node stt_node
```

---

## Notes

- The transcript is only published when an endpoint is detected (trailing silence thresholds). Partial/incomplete utterances are buffered internally.
- Audio is buffered in a `queue.Queue` and processed by a worker thread, keeping the ROS subscriber callback non-blocking.
- There are no configurable ROS parameters beyond what Sherpa-ONNX exposes internally. To tune endpoint sensitivity, edit the `rule*_min_trailing_silence` values directly in the node source.
- The recogniser is English-only (Zipformer EN 2023 model). Swap the model files and adjust paths/settings for other languages.
