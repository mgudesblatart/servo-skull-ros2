# STT Speaking Gate and HALT Re-Enable Plan

Date: 2026-04-27

## Purpose

Capture the current anti-echo STT behavior and document a safe path to re-enable voice interrupt words (for example HALT) in the future.

This note is a continuation artifact so we do not lose implementation context between sessions.

## Problem Summary

Observed runtime issue:
- Skull speech playback was being transcribed by STT after TTS playback, then re-injected as human input.
- This created repeated self-trigger loops and unnecessary LLM turns.

Root cause:
- BT-level gating prevented some downstream effects, but STT itself still decoded and published transcripts during/around speaking windows.

## Implemented Solution (Current State)

### 1) Authoritative speech completion source

Speech completion in BT is now tied to speaker playback completion events, not only chunk timing.

Key details:
- `speaker_node` publishes `/speaker_node/playback_done` after `stream.write(full_audio)` returns.
- `skull_control_bt_node` subscribes and transitions SPEAKING -> IDLE/TRACKING via `tts_done_playback_event`.

Primary files:
- `src/speaker_node/speaker_node/speaker_node.py`
- `src/skull_control_node/skull_control_node/skull_control_bt_node.py`

### 2) STT hard mute while speech subsystem is SPEAKING

`stt_node` subscribes to `/skull_control/state_transition` and applies an internal transcription gate:
- On `subsystem=speech` and `to_state=SPEAKING`:
  - disable transcription
  - drop incoming mic frames
  - flush queued audio
  - reset recognizer stream
- On `subsystem=speech` and `to_state=IDLE`:
  - schedule transcription resume (not immediate)

Primary file:
- `src/stt_node/stt_node/stt_node.py`

### 3) Post-speech unmute tail

To absorb residual speaker tail/room echo, STT resumes after a small fixed delay:
- `STT_RESUME_DELAY_SEC = 0.8`

This delay is enforced in the STT gate check (`_is_transcription_enabled`) using a monotonic timestamp.

## Current Behavioral Contract

During speech playback:
- STT should not publish `/speech_to_text/transcript`.

Immediately after speech completion:
- STT remains suppressed for the configured tail duration.

After tail expiry:
- Normal STT transcription resumes.

## Validation Checklist

Live validation (ROS runtime):
1. Launch full pipeline.
2. Echo:
   - `/skull_control/state_transition`
   - `/speech_to_text/transcript`
   - optional: `/speaker_node/playback_done`
3. Trigger an LLM response long enough to ensure audible playback.
4. Confirm:
   - SPEAKING transitions align with playback completion event.
   - No transcript is published while speech is in SPEAKING.
   - No immediate echo transcript appears during unmute tail.

## Known Trade-offs

- Aggressive STT muting can miss true user barge-in attempts during playback.
- This is intentional for current stability and loop prevention.
- Interrupt handling is deferred to a dedicated future design.

## Future: Safe HALT Re-Enable (Proposed)

Goal: permit reliable voice interrupt commands without reintroducing echo-triggered false positives.

### Phase A: Split interrupt lane from normal transcript lane

- Keep normal transcript decode gated during SPEAKING.
- Add a narrow interrupt detector path active during SPEAKING.
- Detector should only listen for a tiny command set (for example HALT, STOP, CANCEL).

### Phase B: Confidence and anti-echo guards

- Require exact token match at utterance start.
- Add command cooldown/debounce (for example 1-2 s).
- Increase threshold when recent speaker playback just occurred.
- Optionally require repeated confirmation pattern if confidence is low.

### Phase C: Direct control routing

- Route confirmed interrupts directly to control topics:
  - `/tts_node/control` (STOP)
  - `/speaker_node/control` (STOP)
  - `/llm_agent/control` (CANCEL)
- Do not forward interrupt commands as normal LLM transcripts.

### Phase D: Observability and rollback safety

- Emit explicit metrics/log counters:
  - interrupt_detected
  - interrupt_rejected_echo
  - interrupt_executed
- Keep a feature flag/parameter to disable voice interrupts quickly if false positives recur.

## Suggested Next Increment

If/when we resume interrupt work, start with:
1. Feature flag for interrupt lane (default off).
2. Exact-match HALT detector during SPEAKING only.
3. Direct STOP/CANCEL control publish path.
4. Cooldown + structured logs.

This gives controlled capability without touching the stable transcript path.
