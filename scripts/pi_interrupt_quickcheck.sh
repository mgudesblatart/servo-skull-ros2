#!/usr/bin/env bash
set -euo pipefail

# Quick interrupt-only probe for Task 8 behavior.
# Event-driven: uses state transition events to sequence the test rather than fixed timers.
# Assumes the full pipeline is already running.
#
# Checks after publishing HALT to /speech_to_text/transcript:
# - /tts_node/control receives STOP
# - /speaker_node/control receives STOP
# - /llm_agent_axcl/control receives CANCEL
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

print_step "Preflight"
for topic in \
  "${TRANSITION_TOPIC}" \
  "${TTS_CONTROL_TOPIC}" \
  "${SPEAKER_CONTROL_TOPIC}" \
  "${LLM_CONTROL_TOPIC}" \
  "${TEST_EVENT_TOPIC}"
do
  if ! wait_for_topic "${topic}" "${TOPIC_WAIT_SEC}"; then
    echo "FAIL: required topic not available: ${topic}"
    echo "Hint: start the full pipeline with enable_test_events=true, then retry."
    exit 1
  fi
done

print_step "Arming capture listeners"
# All listeners start before any state changes. Generous backup timeouts;
# they are killed explicitly after the terminal event is observed.
timeout 45s ros2 topic echo "${TTS_CONTROL_TOPIC}"     --once > "${TTS_CTRL_LOG}"     2>&1 & TTS_PID=$!
timeout 45s ros2 topic echo "${SPEAKER_CONTROL_TOPIC}" --once > "${SPEAKER_CTRL_LOG}" 2>&1 & SPEAKER_PID=$!
timeout 45s ros2 topic echo "${LLM_CONTROL_TOPIC}"     --once > "${LLM_CTRL_LOG}"     2>&1 & LLM_PID=$!
timeout 45s ros2 topic echo "${TRANSITION_TOPIC}"              > "${TRACE_LOG}"        2>&1 & TRACE_PID=$!
sleep 0.5  # Let subscriptions register before firing any events

print_step "Precondition: force SPEAKING"
# Listen first, then publish — avoids missing a fast transition.
set +o pipefail
timeout "${SPEAKING_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null \
  | grep -m1 'test_event_force_speaking' > /dev/null &
set -o pipefail
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
set +o pipefail
timeout "${INTERRUPT_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null \
  | grep -m1 'interrupt_detected' > /dev/null &
set -o pipefail
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
