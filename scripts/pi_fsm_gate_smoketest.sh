#!/usr/bin/env bash
set -euo pipefail

# Pi smoke test for Task 8 gate + interrupt behavior.
# Assumes full pipeline is already running.
#
# What this verifies:
# 1) While SPEAKING, normal STT transcript is blocked from /skull_control/llm_input.
# 2) HALT interrupt emits STOP/CANCEL control commands and transition trace contains interrupt_detected.
#
# Usage:
#   ./scripts/pi_fsm_gate_smoketest.sh
# Optional env:
#   SPEAKING_WAIT_SEC=25 GATE_CAPTURE_SEC=3 INTERRUPT_CAPTURE_SEC=5 ./scripts/pi_fsm_gate_smoketest.sh
#   TOPIC_WAIT_SEC=10 ./scripts/pi_fsm_gate_smoketest.sh
#   PRIME_SPEAKING=1 PRIME_TEXT='Longer phrase to keep speaking active for gate probe.' ./scripts/pi_fsm_gate_smoketest.sh

SPEAKING_WAIT_SEC="${SPEAKING_WAIT_SEC:-25}"
GATE_CAPTURE_SEC="${GATE_CAPTURE_SEC:-4}"
INTERRUPT_CAPTURE_SEC="${INTERRUPT_CAPTURE_SEC:-10}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-10}"
PRIME_SPEAKING="${PRIME_SPEAKING:-1}"
SKIP_SPEAKING_WAIT="${SKIP_SPEAKING_WAIT:-1}"
PRIME_TEXT="${PRIME_TEXT:-This is a longer speech prime intended to keep speaking active while the gate probe transcript is injected.}"
PRIME_TRANSCRIPT="${PRIME_TRANSCRIPT:-hello there}"

TRANSITION_TOPIC="/skull_control/state_transition"
TRACK_TOPIC="/person_tracking/tracked_persons"
RAW_STT_TOPIC="/speech_to_text/transcript"
LLM_INPUT_TOPIC="/skull_control/llm_input"
TTS_INPUT_TOPIC="/text_to_speech/text_input"
TEST_EVENT_TOPIC="/skull_control/test_event"
TTS_CONTROL_TOPIC="/tts_node/control"
SPEAKER_CONTROL_TOPIC="/speaker_node/control"
LLM_CONTROL_TOPIC="/llm_agent_axcl/control"

BLOCK_TOKEN="GATE_BLOCK_TEST_$(date +%s)"
INTERRUPT_TOKEN="HALT"
RESULT="FAIL"

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $cmd"
    exit 2
  fi
}

cleanup() {
  if [[ -z "${TMP_DIR:-}" || ! -d "${TMP_DIR}" ]]; then
    return
  fi
  if [[ "${RESULT}" == "PASS" ]]; then
    rm -rf "${TMP_DIR}"
    return
  fi
  echo
  echo "Logs preserved for debugging: ${TMP_DIR}"
}
trap cleanup EXIT

require_cmd ros2
require_cmd timeout
require_cmd grep
require_cmd awk

wait_for_topic() {
  local topic="$1"
  local wait_sec="$2"
  local elapsed=0
  while [[ "${elapsed}" -lt "${wait_sec}" ]]; do
    if ros2 topic list 2>/dev/null | awk '{print $1}' | grep -Fxq "${topic}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

TMP_DIR="$(mktemp -d)"
TRACE_LOG="${TMP_DIR}/transition.log"
GATE_LOG="${TMP_DIR}/llm_input_gate.log"
TTS_CTRL_LOG="${TMP_DIR}/tts_control.log"
SPEAKER_CTRL_LOG="${TMP_DIR}/speaker_control.log"
LLM_CTRL_LOG="${TMP_DIR}/llm_control.log"
INT_TRACE_LOG="${TMP_DIR}/interrupt_trace.log"

print_step() {
  echo
  echo "== $1 =="
}

print_step "Preflight"
echo "Watching transition topic: ${TRANSITION_TOPIC}"
echo "Blocked token candidate: ${BLOCK_TOKEN}"
echo "Interrupt token: ${INTERRUPT_TOKEN}"
for required_topic in \
  "${TRANSITION_TOPIC}" \
  "${LLM_INPUT_TOPIC}" \
  "${TTS_CONTROL_TOPIC}" \
  "${SPEAKER_CONTROL_TOPIC}" \
  "${LLM_CONTROL_TOPIC}"
do
  if ! wait_for_topic "${required_topic}" "${TOPIC_WAIT_SEC}"; then
    echo "FAIL: required topic not available: ${required_topic}"
    echo "Hint: bring up the full pipeline, then retry."
    exit 1
  fi
done

print_step "Wait for active speech"
if [[ "${PRIME_SPEAKING}" == "1" ]]; then
  echo "Priming speaking via ${TEST_EVENT_TOPIC} before gate probe."
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null || true
  sleep 0.4
else
  echo "Trigger speech now (normal user interaction), then this script will probe the gate."
fi
if [[ "${SKIP_SPEAKING_WAIT}" != "1" ]]; then
  if timeout "${SPEAKING_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null | grep -m1 '"subsystem": "speech".*"to": "SPEAKING"' >"${TRACE_LOG}"; then
    echo "PASS: observed speech transition to SPEAKING"
  else
    echo "FAIL: did not observe SPEAKING transition within ${SPEAKING_WAIT_SEC}s"
    echo "Tip: make sure BT + TTS + speaker are running and produce speech before retrying."
    exit 1
  fi
fi

print_step "Gate probe during speech"
if [[ "${PRIME_SPEAKING}" == "1" ]]; then
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null || true
  sleep 0.2
fi
# Capture llm input stream while sending a normal transcript that should be blocked.
timeout "${GATE_CAPTURE_SEC}s" ros2 topic echo "${LLM_INPUT_TOPIC}" >"${GATE_LOG}" 2>&1 &
GATE_PID=$!
sleep 1.0
ros2 topic pub --once "${RAW_STT_TOPIC}" std_msgs/String "data: '${BLOCK_TOKEN}'" >/dev/null
wait "${GATE_PID}" || true

if grep -q "${BLOCK_TOKEN}" "${GATE_LOG}"; then
  echo "FAIL: blocked transcript appeared on ${LLM_INPUT_TOPIC}"
  echo "Evidence:"
  sed -n '1,120p' "${GATE_LOG}"
  exit 1
else
  echo "PASS: blocked transcript did not appear on ${LLM_INPUT_TOPIC}"
fi

print_step "Interrupt probe"
if [[ "${PRIME_SPEAKING}" == "1" ]]; then
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null || true
  sleep 0.2
fi
timeout "${INTERRUPT_CAPTURE_SEC}s" ros2 topic echo "${TTS_CONTROL_TOPIC}" --once >"${TTS_CTRL_LOG}" 2>&1 &
TTS_PID=$!
timeout "${INTERRUPT_CAPTURE_SEC}s" ros2 topic echo "${SPEAKER_CONTROL_TOPIC}" --once >"${SPEAKER_CTRL_LOG}" 2>&1 &
SPEAKER_PID=$!
timeout "${INTERRUPT_CAPTURE_SEC}s" ros2 topic echo "${LLM_CONTROL_TOPIC}" --once >"${LLM_CTRL_LOG}" 2>&1 &
LLM_PID=$!
timeout "${INTERRUPT_CAPTURE_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" >"${INT_TRACE_LOG}" 2>&1 &
TRACE_PID=$!

sleep 1.0
for _ in 1 2 3; do
  ros2 topic pub --once "${RAW_STT_TOPIC}" std_msgs/String "data: '${INTERRUPT_TOKEN}'" >/dev/null
  sleep 0.3
done

wait "${TTS_PID}" || true
wait "${SPEAKER_PID}" || true
wait "${LLM_PID}" || true
wait "${TRACE_PID}" || true

PASS_COUNT=0

if grep -q 'data: STOP' "${TTS_CTRL_LOG}"; then
  echo "PASS: tts control STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: tts control STOP not observed"
fi

if grep -q 'data: STOP' "${SPEAKER_CTRL_LOG}"; then
  echo "PASS: speaker control STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: speaker control STOP not observed"
fi

if grep -q 'data: CANCEL' "${LLM_CTRL_LOG}"; then
  echo "PASS: llm control CANCEL observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: llm control CANCEL not observed"
fi

if grep -q '"event": "interrupt_detected"' "${INT_TRACE_LOG}"; then
  echo "PASS: interrupt_detected transition observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: interrupt_detected transition not observed"
fi

print_step "Summary"
echo "Checks passed: ${PASS_COUNT}/4"
if [[ "${PASS_COUNT}" -lt 4 ]]; then
  echo "Result: FAIL"
  exit 1
fi

RESULT="PASS"
echo "Result: PASS"
