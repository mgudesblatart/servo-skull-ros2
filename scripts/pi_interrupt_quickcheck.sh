#!/usr/bin/env bash
set -euo pipefail

# Quick interrupt-only probe for Task 8 behavior.
# Event-driven: uses state transition events to sequence the test rather than fixed timers.
# Assumes the full pipeline is already running.
#
# Checks after publishing HALT to /speech_to_text/transcript:
# - /tts_node/control receives STOP
# - /speaker_node/control receives STOP
# - /llm_agent/control receives CANCEL
# - /skull_control/state_transition may include interrupt_detected (advisory)
#
# Usage:
#   ./scripts/pi_interrupt_quickcheck.sh
# Optional env:
#   SPEAKING_WAIT_SEC=20 INTERRUPT_WAIT_SEC=10 ./scripts/pi_interrupt_quickcheck.sh

SPEAKING_WAIT_SEC="${SPEAKING_WAIT_SEC:-20}"
INTERRUPT_WAIT_SEC="${INTERRUPT_WAIT_SEC:-10}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-10}"
RAW_STT_TOPIC="/speech_to_text/transcript"
TEST_EVENT_TOPIC="/skull_control/test_event"
TTS_CONTROL_TOPIC="/tts_node/control"
SPEAKER_CONTROL_TOPIC="/speaker_node/control"
LLM_CONTROL_TOPIC="/llm_agent/control"
TRANSITION_TOPIC="/skull_control/state_transition"

STATE_TRANSITION_TYPE="servo_skull_msgs/msg/StateTransition"
STRING_TYPE="std_msgs/msg/String"

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

wait_for_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local wait_sec="$3"
  local elapsed=0
  local actual_type=""

  while [[ "${elapsed}" -lt "${wait_sec}" ]]; do
    actual_type="$(ros2 topic type "${topic}" 2>/dev/null | head -n1 | tr -d '[:space:]')"
    if [[ -n "${actual_type}" ]]; then
      if [[ "${actual_type}" == "${expected_type}" ]]; then
        return 0
      fi
      echo "FAIL: topic ${topic} has unexpected type ${actual_type} (expected ${expected_type})"
      return 1
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "FAIL: topic type not available for ${topic} within ${wait_sec}s"
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

print_step "Preflight"
for required_topic in \
  "${TRANSITION_TOPIC}|${STATE_TRANSITION_TYPE}" \
  "${RAW_STT_TOPIC}|${STRING_TYPE}" \
  "${TTS_CONTROL_TOPIC}|${STRING_TYPE}" \
  "${SPEAKER_CONTROL_TOPIC}|${STRING_TYPE}" \
  "${LLM_CONTROL_TOPIC}|${STRING_TYPE}" \
  "${TEST_EVENT_TOPIC}|${STRING_TYPE}"
do
  IFS='|' read -r topic expected_type <<< "${required_topic}"
  if ! wait_for_topic "${topic}" "${TOPIC_WAIT_SEC}"; then
    echo "FAIL: required topic not available: ${topic}"
    echo "Hint: start the full pipeline with enable_test_events=true, then retry."
    exit 1
  fi
  if ! wait_for_topic_type "${topic}" "${expected_type}" "${TOPIC_WAIT_SEC}"; then
    echo "Hint: start the full pipeline with enable_test_events=true, then retry."
    exit 1
  fi
done

print_step "Arming capture listeners"
# All listeners start before any state changes. Generous backup timeouts;
# they are killed explicitly after the terminal event is observed.
timeout 45s ros2 topic echo "${TTS_CONTROL_TOPIC}"     "${STRING_TYPE}" --field data --once > "${TTS_CTRL_LOG}"     2>&1 & TTS_PID=$!
timeout 45s ros2 topic echo "${SPEAKER_CONTROL_TOPIC}" "${STRING_TYPE}" --field data --once > "${SPEAKER_CTRL_LOG}" 2>&1 & SPEAKER_PID=$!
timeout 45s ros2 topic echo "${LLM_CONTROL_TOPIC}"     "${STRING_TYPE}" --field data --once > "${LLM_CTRL_LOG}"     2>&1 & LLM_PID=$!
timeout 45s ros2 topic echo "${TRANSITION_TOPIC}"      "${STATE_TRANSITION_TYPE}" --field event > "${TRACE_LOG}" 2>&1 & TRACE_PID=$!
sleep 0.5  # Let subscriptions register before firing any events

print_step "Precondition: force SPEAKING"
# Listen first, then publish — avoids missing a fast transition.
timeout "${SPEAKING_WAIT_SEC}s" ros2 topic echo \
  "${TRANSITION_TOPIC}" \
  "${STATE_TRANSITION_TYPE}" \
  --filter "m.event == 'test_event_force_speaking' and m.subsystem == 'speech' and m.to_state == 'SPEAKING'" \
  --once > /dev/null 2>&1 &
SPEAKING_PID=$!
sleep 0.3
for _ in 1 2 3; do
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null
  sleep 0.2
done
if wait "${SPEAKING_PID}"; then
  echo "Observed speech->SPEAKING"
else
  echo "FAIL: did not observe SPEAKING transition within ${SPEAKING_WAIT_SEC}s"
  kill "${TTS_PID}" "${SPEAKER_PID}" "${LLM_PID}" "${TRACE_PID}" 2>/dev/null || true
  exit 1
fi

print_step "Publishing HALT"
# Arm interrupt gate first (listen-first), then publish immediately.
timeout "${INTERRUPT_WAIT_SEC}s" ros2 topic echo \
  "${TRANSITION_TOPIC}" \
  "${STATE_TRANSITION_TYPE}" \
  --filter "m.event == 'interrupt_detected' and m.subsystem == 'speech' and m.to_state == 'IDLE'" \
  --once > /dev/null 2>&1 &
INTERRUPT_PID=$!
sleep 0.2
ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null
sleep 0.2
ros2 topic pub --once "${RAW_STT_TOPIC}" std_msgs/String "data: '${INTERRUPT_TOKEN}'" >/dev/null

if wait "${INTERRUPT_PID}"; then
  echo "Observed interrupt_detected"
else
  echo "WARNING: interrupt_detected not seen within ${INTERRUPT_WAIT_SEC}s"
fi

# Let control topic listeners drain any in-flight messages, then clean up.
sleep 0.3
kill "${TTS_PID}" "${SPEAKER_PID}" "${LLM_PID}" "${TRACE_PID}" 2>/dev/null || true
wait "${TTS_PID}" "${SPEAKER_PID}" "${LLM_PID}" "${TRACE_PID}" 2>/dev/null || true

PASS_COUNT=0

if grep -Fxq 'STOP' "${TTS_CTRL_LOG}"; then
  echo "PASS: TTS STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: TTS STOP not observed"
fi

if grep -Fxq 'STOP' "${SPEAKER_CTRL_LOG}"; then
  echo "PASS: speaker STOP observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: speaker STOP not observed"
fi

if grep -Fxq 'CANCEL' "${LLM_CTRL_LOG}"; then
  echo "PASS: LLM CANCEL observed"
  PASS_COUNT=$((PASS_COUNT + 1))
else
  echo "FAIL: LLM CANCEL not observed"
fi

if grep -q 'interrupt_detected' "${TRACE_LOG}"; then
  echo "INFO: interrupt_detected transition observed"
else
  echo "INFO: interrupt_detected transition not observed (non-fatal)"
fi

print_step "Summary"
echo "Required checks passed: ${PASS_COUNT}/3"
if [[ "${PASS_COUNT}" -lt 3 ]]; then
  echo "Result: FAIL"
  exit 1
fi

RESULT="PASS"
echo "Result: PASS"
