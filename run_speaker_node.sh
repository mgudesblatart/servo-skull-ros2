#!/bin/bash
# Run the speaker_node and tts_node directly with ros2 run, using your workspace's Python env

set -e

# Activate your virtualenv if needed
source ~/projects/venv-servo-skull/bin/activate

# Source ROS2 and workspace overlays
source /opt/ros/jazzy/setup.bash
source ~/projects/servo-skull/install/setup.bash

# Start speaker_node in background
ros2 run speaker_node speaker_node &
SPEAKER_PID=$!
# Start tts_node in background
ros2 run tts_node tts_node &
TTS_PID=$!

# Give nodes time to start
sleep 2

# Send a test message to tts_node
ros2 topic pub --once /text_to_speech/text_input std_msgs/String '{data: "Hello, servo skull! This is a test of the TTS and speaker nodes."}'

# Wait for playback to finish
sleep 5

# Kill background nodes
kill $TTS_PID $SPEAKER_PID
wait $TTS_PID $SPEAKER_PID 2>/dev/null || true
