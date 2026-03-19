#!/usr/bin/env bash
set -euo pipefail

# Pi smoke test for Task 8 gate + interrupt behavior.
# Event-driven: uses state transition events to sequence the test rather than fixed timers.
# Assumes full pipeline is already running.
#
# What this verifies:
# 1) While SPEAKING, normal STT transcript is blocked from /skull_control/llm_input.
# 2) HALT interrupt emits STOP/CANCEL control commands; interrupt transition trace is advisory.
#
# Usage:
#   ./scripts/pi_fsm_gate_smoketest.sh
# Optional env:
#   SPEAKING_WAIT_SEC=20 INTERRUPT_WAIT_SEC=10 IDLE_WAIT_SEC=15 ./scripts/pi_fsm_gate_smoketest.sh
#   GATE_CAPTURE_SEC=3 TOPIC_WAIT_SEC=10 ./scripts/pi_fsm_gate_smoketest.sh

SPEAKING_WAIT_SEC="${SPEAKING_WAIT_SEC:-20}"
GATE_CAPTURE_SEC="${GATE_CAPTURE_SEC:-3}"   # Short window for negative gate check; inherently timer-based
INTERRUPT_WAIT_SEC="${INTERRUPT_WAIT_SEC:-10}"
IDLE_WAIT_SEC="${IDLE_WAIT_SEC:-15}"         # Wait for watchdog to fire between gate and interrupt phases
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-10}"

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
echo "Blocked token: ${BLOCK_TOKEN}"
echo "Interrupt token: ${INTERRUPT_TOKEN}"
for required_topic in \
  "${TRANSITION_TOPIC}" \
  "${LLM_INPUT_TOPIC}" \
  "${TTS_CONTROL_TOPIC}" \
  "${SPEAKER_CONTROL_TOPIC}" \
  "${LLM_CONTROL_TOPIC}" \
  "${TEST_EVENT_TOPIC}"
do
  if ! wait_for_topic "${required_topic}" "${TOPIC_WAIT_SEC}"; then
    echo "FAIL: required topic not available: ${required_topic}"
    echo "Hint: bring up the full pipeline with enable_test_events=true, then retry."
    exit 1
  fi
done

# ---------------------------------------------------------------------------
# Phase 1: Gate probe
# Force SPEAKING, inject a normal transcript, confirm it doesn't reach LLM input.
# ---------------------------------------------------------------------------
print_step "Gate probe: force SPEAKING"
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
  echo "FAIL: no SPEAKING transition within ${SPEAKING_WAIT_SEC}s"
  exit 1
fi

print_step "Gate probe: publish BLOCK_TOKEN while SPEAKING"
timeout 45s ros2 topic echo "${LLM_INPUT_TOPIC}" > "${GATE_LOG}" 2>&1 & GATE_PID=$!
sleep 0.5
ros2 topic pub --once "${RAW_STT_TOPIC}" std_msgs/String "data: '${BLOCK_TOKEN}'" >/dev/null
# Negative check: short fixed window is fine here — if the token was going to
# slip through, it would arrive within milliseconds of being published.
sleep "${GATE_CAPTURE_SEC}"
kill "${GATE_PID}" 2>/dev/null || true
wait "${GATE_PID}" 2>/dev/null || true

if grep -q "${BLOCK_TOKEN}" "${GATE_LOG}"; then
  echo "FAIL: blocked transcript appeared on ${LLM_INPUT_TOPIC}"
  echo "Evidence:"
  sed -n '1,20p' "${GATE_LOG}"
  exit 1
else
  echo "PASS: blocked transcript did not appear on ${LLM_INPUT_TOPIC}"
fi

# ---------------------------------------------------------------------------
# Wait for node to return to IDLE between phases.
# The speaking stall watchdog will fire within SPEAKING_STALL_TIMEOUT_SEC.
# We need IDLE before we can trigger a fresh SPEAKING for the interrupt probe.
# ---------------------------------------------------------------------------
print_step "Waiting for node to return to IDLE"
set +o pipefail
timeout "${IDLE_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null \
  | grep -m1 'speaking_stall_timeout' > /dev/null &
set -o pipefail
IDLE_PID=$!
if wait "${IDLE_PID}"; then
  echo "Observed general->IDLE"
else
  echo "WARNING: IDLE not observed within ${IDLE_WAIT_SEC}s; proceeding anyway"
fi

# ---------------------------------------------------------------------------
# Phase 2: Interrupt probe
# Force SPEAKING again, publish HALT, confirm control commands + transition.
# ---------------------------------------------------------------------------
print_step "Interrupt probe: arming capture listeners"
timeout 45s ros2 topic echo "${TTS_CONTROL_TOPIC}"     --once > "${TTS_CTRL_LOG}"     2>&1 & TTS_PID=$!
timeout 45s ros2 topic echo "${SPEAKER_CONTROL_TOPIC}" --once > "${SPEAKER_CTRL_LOG}" 2>&1 & SPEAKER_PID=$!
timeout 45s ros2 topic echo "${LLM_CONTROL_TOPIC}"     --once > "${LLM_CTRL_LOG}"     2>&1 & LLM_PID=$!
timeout 45s ros2 topic echo "${TRANSITION_TOPIC}"              > "${INT_TRACE_LOG}"    2>&1 & TRACE_PID=$!
sleep 0.5

print_step "Interrupt probe: force SPEAKING"
set +o pipefail
timeout "${SPEAKING_WAIT_SEC}s" ros2 topic echo "${TRANSITION_TOPIC}" 2>/dev/null \
  | grep -m1 'test_event_force_speaking' > /dev/null &
set -o pipefail
SPEAKING_PID2=$!
sleep 0.3
for _ in 1 2 3; do
  ros2 topic pub --once "${TEST_EVENT_TOPIC}" std_msgs/String "data: 'FORCE_SPEAKING'" >/dev/null
  sleep 0.2
done
if wait "${SPEAKING_PID2}"; then
  echo "Observed speech->SPEAKING"
else
  echo "WARNING: no SPEAKING transition for interrupt phase within ${SPEAKING_WAIT_SEC}s"
  echo "Continuing interrupt probe; control outputs are authoritative for pass/fail."
fi

print_step "Interrupt probe: publishing HALT"
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

# Let control topic listeners drain, then clean up.
sleep 0.3
kill "${TTS_PID}" "${SPEAKER_PID}" "${LLM_PID}" "${TRACE_PID}" 2>/dev/null || true
wait "${TTS_PID}" "${SPEAKER_PID}" "${LLM_PID}" "${TRACE_PID}" 2>/dev/null || true

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

if grep -q 'interrupt_detected' "${INT_TRACE_LOG}"; then
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
