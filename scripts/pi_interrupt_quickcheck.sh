#!/usr/bin/env bash
set -euo pipefail

# Quick interrupt-only probe for Task 8 behavior.
# Assumes the full pipeline is already running.
#
# Checks after publishing HALT to /speech_to_text/transcript:
# - /tts_node/control receives STOP
# - /speaker_node/control receives STOP
# - /llm_agent_axcl/control receives CANCEL
# - /skull_control/state_transition includes interrupt_detected
#
# Usage:
#   ./scripts/pi_interrupt_quickcheck.sh
# Optional env:
#   CAPTURE_SEC=6 SPEAKING_WAIT_SEC=25 ./scripts/pi_interrupt_quickcheck.sh
#   SKIP_SPEAKING_WAIT=1 ./scripts/pi_interrupt_quickcheck.sh
#   PRIME_SPEAKING=1 PRIME_TEXT='Quick speech prime for interrupt test.' ./scripts/pi_interrupt_quickcheck.sh

CAPTURE_SEC="${CAPTURE_SEC:-10}"
SPEAKING_WAIT_SEC="${SPEAKING_WAIT_SEC:-20}"
SKIP_SPEAKING_WAIT="${SKIP_SPEAKING_WAIT:-1}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-10}"
PRIME_SPEAKING="${PRIME_SPEAKING:-1}"
RAW_STT_TOPIC="/speech_to_text/transcript"
TEST_EVENT_TOPIC="/skull_control/test_event"
TTS_CONTROL_TOPIC="/tts_node/control"
SPEAKER_CONTROL_TOPIC="/speaker_node/control"
LLM_CONTROL_TOPIC="/llm_agent_axcl/control"
TRANSITION_TOPIC="/skull_control/state_transition"
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
TTS_CTRL_LOG="${TMP_DIR}/tts_control.log"
SPEAKER_CTRL_LOG="${TMP_DIR}/speaker_control.log"
LLM_CTRL_LOG="${TMP_DIR}/llm_control.log"
TRACE_LOG="${TMP_DIR}/transition.log"

print_step() {
  echo
  echo "== $1 =="
}

print_step "Capturing control/transition topics"
if ! wait_for_topic "${TRANSITION_TOPIC}" "${TOPIC_WAIT_SEC}"; then
  echo "FAIL: required topic not available: ${TRANSITION_TOPIC}"
  echo "Hint: start skull_control_bt_node and full pipeline, then retry."
  exit 1
fi
if ! wait_for_topic "${TTS_CONTROL_TOPIC}" "${TOPIC_WAIT_SEC}"; then
  echo "FAIL: required topic not available: ${TTS_CONTROL_TOPIC}"
  echo "Hint: start tts_node and verify BT is running."
  exit 1
fi
if ! wait_for_topic "${SPEAKER_CONTROL_TOPIC}" "${TOPIC_WAIT_SEC}"; then
  echo "FAIL: required topic not available: ${SPEAKER_CONTROL_TOPIC}"
  echo "Hint: start speaker_node and verify BT is running."
  exit 1
fi
if ! wait_for_topic "${LLM_CONTROL_TOPIC}" "${TOPIC_WAIT_SEC}"; then
  echo "FAIL: required topic not available: ${LLM_CONTROL_TOPIC}"
  echo "Hint: start llm_agent_axcl_node and verify BT is running."
  exit 1
fi

timeout "${CAPTURE_SEC}s" ros2 topic echo "${TTS_CONTROL_TOPIC}" --once >"${TTS_CTRL_LOG}" 2>&1 &
TTS_PID=$!
timeout "${CAPTURE_SEC}s" ros2 topic echo "${SPEAKER_CONTROL_TOPIC}" --once >"${SPEAKER_CTRL_LOG}" 2>&1 &
SPEAKER_PID=$!
timeout "${CAPTURE_SEC}s" ros2 topic echo "${LLM_CONTROL_TOPIC}" --once >"${LLM_CTRL_LOG}" 2>&1 &
LLM_PID=$!
timeout "${CAPTURE_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" >"${TRACE_LOG}" 2>&1 &
TRACE_PID=$!

if [[ "${PRIME_SPEAKING}" == "1" ]]; then
  print_step "Priming speaking"
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null || true
  sleep 0.3
fi

if [[ "${SKIP_SPEAKING_WAIT}" != "1" ]]; then
  print_step "Precondition: wait for active speech"
  echo "Trigger speech now; waiting up to ${SPEAKING_WAIT_SEC}s for speech->SPEAKING transition"
  if timeout "${SPEAKING_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null | grep -m1 '"subsystem": "speech".*"to": "SPEAKING"' >/dev/null; then
    echo "Observed speech transition to SPEAKING"
  else
    echo "FAIL: did not observe SPEAKING transition in time"
    exit 1
  fi
fi

sleep 1.0
print_step "Publishing HALT"
if [[ "${PRIME_SPEAKING}" == "1" ]]; then
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null || true
  sleep 0.2
fi
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
  echo "PASS: TTS STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: TTS STOP not observed"
fi

if grep -q 'data: STOP' "${SPEAKER_CTRL_LOG}"; then
  echo "PASS: speaker STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: speaker STOP not observed"
fi

if grep -q 'data: CANCEL' "${LLM_CTRL_LOG}"; then
  echo "PASS: LLM CANCEL observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: LLM CANCEL not observed"
fi

if grep -q '"event": "interrupt_detected"' "${TRACE_LOG}"; then
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
