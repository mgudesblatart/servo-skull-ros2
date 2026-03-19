# Task 8 State Machine Spec (v1)

Date: 2026-03-18

## Constants

- LOW_INTEREST_TIMEOUT_S = 60.0
- NO_MOTION_TIMEOUT_S = 30.0
- NO_SPEECH_TIMEOUT_S = 30.0
- DISPLAY_EMOTION_TIMEOUT_S = 2.5
- INTERRUPT_TOKENS = {"HALT", "HOLD"}

## State Sets

- GeneralState: IDLE, TRACKING, THINKING, SPEAKING
- EyeState: IDLE, TRACKING, THINKING, DISPLAY_EMOTION, BORED
- LLMState: IDLE, THINKING, SPEAKING
- SpeechState: IDLE, SPEAKING

## Core Transition Rules

1. IDLE -> TRACKING on person_detected.
2. TRACKING -> THINKING on speech_detected.
3. THINKING -> SPEAKING on verbal response ready.
4. THINKING -> TRACKING on non-verbal response and Eye -> DISPLAY_EMOTION.
5. SPEAKING -> SPEAKING while TTS active.
6. SPEAKING -> TRACKING when TTS done (or IDLE if no target).
7. TRACKING -> BORED (Eye substate) on low_interest_timeout.
8. TRACKING -> IDLE on no_speech_timeout and no_motion_timeout.

## Overlay and Priority Rules

- Eye DISPLAY_EMOTION is an overlay substate with timeout.
- While Eye DISPLAY_EMOTION is active, default Eye TRACKING updates do not override it.
- Eye BORED exits immediately to Eye TRACKING on new motion/person activity.

## STT Gating and Interrupts

- While SpeechState is SPEAKING, ignore normal STT input.
- Exception: if transcript contains an interrupt token (HALT/HOLD), then:
  1) stop TTS immediately,
  2) cancel active LLM reasoning,
  3) transition back to General TRACKING.

## Prompt Constraint for LLM

System prompt should include:

"Reserved control tokens: HALT, HOLD. Never output these tokens in normal assistant responses. If needed, paraphrase without these exact tokens."

## Implementation Notes

- Add explicit state ownership and transition logging.
- Log transitions with from/to/event/guard/timestamp.
- Keep the existing TrackSelector + PanTiltPublisher path as the initial ENGAGE/TRACKING behavior foundation.
