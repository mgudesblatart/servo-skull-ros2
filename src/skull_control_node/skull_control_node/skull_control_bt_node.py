#!/usr/bin/env python3
"""
skull_control_bt_node.py

The central coordination hub for the servo skull. Owns the FSM (Finite State
Machine) that governs what the skull is doing at any moment, drives the
py_trees behavior tree for pan/tilt tracking, and routes signals between the
STT, TTS, LLM, and speaker subsystems.

Topic wiring overview:
  Subscribed:
    /person_tracking/tracked_persons  -> target selection + motion activity
    /speech_to_text/transcript        -> user speech + interrupt detection
    /text_to_speech/text_input        -> watches outgoing TTS to arm echo suppression
    /speaker/audio_output             -> detects last audio chunk to end SPEAKING state
    /skull_control/test_event         -> (optional) force FSM states for testing

  Published:
    skull_control/pan_tilt_cmd        -> servo pan/tilt setpoint
    /skull_control/state_transition   -> JSON trace of every FSM transition
    /skull_control/llm_input          -> gated transcripts + system events -> LLM agent
    /tts_node/control                 -> STOP command on interrupt
    /speaker_node/control             -> STOP command on interrupt
    /llm_agent/control                -> CANCEL command on interrupt
"""

import json
import re
import time
from enum import Enum

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from servo_skull_msgs.msg import AudioData, TrackedPersons
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Coordinate mapping: person-sensor pixel space -> ESP32 servo range
# ---------------------------------------------------------------------------
SENSOR_MIN = 0
SENSOR_MAX = 255
ESP32_MIN = 0
ESP32_MAX = 500

# How long (seconds) to keep tracking a person after they disappear from frame
TARGET_TIMEOUT_SEC = 5

# ---------------------------------------------------------------------------
# FSM timeout thresholds
# ---------------------------------------------------------------------------
# How long a tracked person can be silent/still before we call it "bored"
LOW_INTEREST_TIMEOUT_SEC = 60.0
# No detected motion before nudging the LLM with a system event
NO_MOTION_TIMEOUT_SEC = 30.0
# No speech before nudging the LLM with a system event
NO_SPEECH_TIMEOUT_SEC = 30.0
# How long the eye holds a DISPLAY_EMOTION overlay before reverting
DISPLAY_EMOTION_TIMEOUT_SEC = 2.5
# Safety net: if speaker goes silent mid-utterance for this long, assume done
SPEAKING_STALL_TIMEOUT_SEC = 8.0
# Safety net: if LLM never produces TTS while THINKING, recover FSM
THINKING_STALL_TIMEOUT_SEC = 75.0

# ---------------------------------------------------------------------------
# Echo suppression: prevents the skull hearing its own voice as user speech
# ---------------------------------------------------------------------------
# Hard time-based blackout window right after TTS speech ends.
# In live runs, speaker playback can continue for several seconds after
# tts_done bookkeeping transitions, so keep STT gated longer.
POST_TTS_ECHO_SUPPRESS_SEC = 9.0
# How long after a TTS phrase to keep checking if incoming STT looks like it.
# Keep this comfortably above observed speaker tail latency.
ECHO_TEXT_MATCH_WINDOW_SEC = 15.0
# Jaccard similarity above this threshold = probably an echo
ECHO_TEXT_SIMILARITY_THRESHOLD = 0.55

# Spoken words that immediately halt all activity and reset state
INTERRUPT_TOKENS = {'HALT', 'HOLD'}

# ---------------------------------------------------------------------------
# System event debounce: rate-limits how often we spam the LLM with events
# ---------------------------------------------------------------------------
SYSTEM_EVENT_COOLDOWN_DEFAULT_SEC = 10.0
IDENTICAL_EVENT_SUPPRESS_SEC = 30.0  # Extra suppression for exact duplicate payloads
SYSTEM_EVENT_COOLDOWN_SEC = {
    'person_detected': 8.0,
    'no_motion_timeout': 10.0,
    'no_speech_timeout': 10.0,
    'low_interest_timeout': 12.0,
    'interrupt_detected': 5.0,
    'tts_done': 5.0,
}


# ---------------------------------------------------------------------------
# FSM state enums
# Four parallel sub-machines; GeneralState is the "top level" summary.
# ---------------------------------------------------------------------------

class GeneralState(Enum):
    IDLE = 'IDLE'           # No person, nothing happening
    TRACKING = 'TRACKING'  # Person detected, eyes following
    THINKING = 'THINKING'  # LLM is processing speech input
    SPEAKING = 'SPEAKING'  # TTS/speaker is outputting audio


class EyeState(Enum):
    IDLE = 'IDLE'
    TRACKING = 'TRACKING'
    THINKING = 'THINKING'
    DISPLAY_EMOTION = 'DISPLAY_EMOTION'  # Temporary overlay; reverts after timeout
    BORED = 'BORED'                      # Person present but no interaction


class LLMState(Enum):
    IDLE = 'IDLE'
    THINKING = 'THINKING'  # Waiting for model response
    SPEAKING = 'SPEAKING'  # Model responded and TTS is running


class SpeechState(Enum):
    IDLE = 'IDLE'
    SPEAKING = 'SPEAKING'


# ---------------------------------------------------------------------------
# Blackboard: shared state between the BT behaviours and the FSM
# ---------------------------------------------------------------------------

class Blackboard:
    """Flat container for all runtime state. Passed by reference to BT nodes."""

    def __init__(self):
        # Current tracking target (person object) and its ID for sticky re-acquisition
        self.current_target = None
        self.current_target_id = None
        self.target_lost_time = None    # Timestamp when target first disappeared
        self.tracks = []                # Latest TrackedPersons list from person_tracking

        # FSM state machines
        self.general_state = GeneralState.IDLE
        self.eye_state = EyeState.IDLE
        self.llm_state = LLMState.IDLE
        self.speech_state = SpeechState.IDLE

        now = time.monotonic()

        # Activity timestamps used for timeout logic
        self.last_motion_ts = now
        self.last_speech_ts = now
        self.last_interest_ts = now     # Reset on any person/speech activity
        self.last_state_change_ts = now

        # Eye overlay expiry: DISPLAY_EMOTION holds until this timestamp
        self.eye_overlay_until_ts = 0.0

        # Tracks the last received audio chunk timestamp for stall detection
        self.last_audio_chunk_ts = now

        # Echo suppression state
        self.stt_echo_suppress_until_ts = 0.0  # Hard blackout window post-TTS
        self.last_tts_phrase = ''               # Text of the most recent TTS output
        self.last_tts_phrase_ts = 0.0           # When that phrase was sent

        # Per-event debounce tracking for system events published to LLM
        self.last_system_event_publish_ts = {}   # {event_name: monotonic_ts}
        self.last_system_event_signature = {}    # {event_name: json_payload_str}


# ---------------------------------------------------------------------------
# Behavior tree leaf nodes
# ---------------------------------------------------------------------------

class TrackSelector(py_trees.behaviour.Behaviour):
    """
    Picks which person to follow.

    Tries to maintain a "sticky" lock on the current target across frames.
    Falls back to highest-confidence detection if the current target is lost,
    and clears the target entirely after TARGET_TIMEOUT_SEC.

    Returns FAILURE when there's nobody to track (halts the BT sequence).
    """

    def __init__(self, name, blackboard):
        super().__init__(name)
        self.blackboard = blackboard

    def update(self):
        tracks = self.blackboard.tracks
        now = time.monotonic()

        if not tracks:
            # Start or advance the lost-target timeout
            if self.blackboard.current_target_id is not None and self.blackboard.target_lost_time is None:
                self.blackboard.target_lost_time = now
            elif (
                self.blackboard.target_lost_time is not None
                and (now - self.blackboard.target_lost_time) > TARGET_TIMEOUT_SEC
            ):
                self.blackboard.current_target = None
                self.blackboard.current_target_id = None
                self.blackboard.target_lost_time = None
            return py_trees.common.Status.FAILURE

        # Try to re-acquire the previously tracked person
        found = next(
            (p for p in tracks if p.person_id == self.blackboard.current_target_id),
            None,
        )

        if found:
            self.blackboard.current_target = found
            self.blackboard.target_lost_time = None
        else:
            # Current target gone — pick whoever has the highest detection confidence
            best = max(tracks, key=lambda p: p.box_confidence)
            self.blackboard.current_target = best
            self.blackboard.current_target_id = best.person_id
            self.blackboard.target_lost_time = None

        return py_trees.common.Status.SUCCESS


class PanTiltPublisher(py_trees.behaviour.Behaviour):
    """
    Converts the selected target's bounding box center into servo coordinates
    and publishes a pan/tilt command.

    Coordinate mapping: sensor pixel range [0-255] -> ESP32 servo range [0-500].
    """

    def __init__(self, name, blackboard, publisher):
        super().__init__(name)
        self.blackboard = blackboard
        self.publisher = publisher

    def update(self):
        target = self.blackboard.current_target
        if not target:
            return py_trees.common.Status.FAILURE

        x_raw = int((target.box_left + target.box_right) / 2)
        y_raw = int((target.box_top + target.box_bottom) / 2)

        # Linear remap from sensor space to ESP32 servo space
        x = int((x_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
        y = int((y_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)

        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.0
        self.publisher.publish(pt)
        return py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# Main ROS node
# ---------------------------------------------------------------------------

class SkullControlBTNode(Node):
    def __init__(self):
        super().__init__('skull_control_bt_node')

        self.declare_parameter('enable_test_events', False)
        self.enable_test_events = bool(self.get_parameter('enable_test_events').value)

        self.blackboard = Blackboard()

        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_behavior_tree()

        # Main loop: 10 Hz tick drives both the FSM and the BT
        self.tick_timer = self.create_timer(0.1, self._on_tick)

        if self.enable_test_events:
            self.get_logger().warning('Test event hook enabled on /skull_control/test_event')
        self.get_logger().info('Skull Control BT Node started.')

    def _setup_subscriptions(self):
        """Wire up all incoming topic subscriptions."""
        self.subscription = self.create_subscription(
            TrackedPersons,
            '/person_tracking/tracked_persons',
            self.tracked_persons_callback,
            10,
        )
        self.stt_subscription = self.create_subscription(
            String,
            '/speech_to_text/transcript',
            self.stt_callback,
            10,
        )
        self.tts_text_input_subscription = self.create_subscription(
            String,
            '/text_to_speech/text_input',
            self.tts_text_input_callback,
            10,
        )
        self.speaker_audio_subscription = self.create_subscription(
            AudioData,
            '/speaker/audio_output',
            self.speaker_audio_callback,
            10,
        )
        self.llm_status_subscription = self.create_subscription(
            String,
            '/llm_agent/status',
            self.llm_status_callback,
            10,
        )
        # Optional: allows injecting FSM state changes via topic (useful for integration tests)
        self.test_event_subscription = None
        if self.enable_test_events:
            self.test_event_subscription = self.create_subscription(
                String,
                '/skull_control/test_event',
                self.test_event_callback,
                10,
            )

    def _setup_publishers(self):
        """Create all outgoing topic publishers."""
        self.cmd_pub = self.create_publisher(Point, 'skull_control/pan_tilt_cmd', 10)
        # Large queue depth so fast FSM transitions don't get dropped
        self.state_transition_pub = self.create_publisher(String, '/skull_control/state_transition', 50)
        self.llm_input_pub = self.create_publisher(String, '/skull_control/llm_input', 10)
        self.tts_control_pub = self.create_publisher(String, '/tts_node/control', 10)
        self.speaker_control_pub = self.create_publisher(String, '/speaker_node/control', 10)
        self.llm_control_pub = self.create_publisher(String, '/llm_agent/control', 10)

    def _setup_behavior_tree(self):
        """Build the py_trees BT: TrackSelector -> PanTiltPublisher in a sequence."""
        selector = TrackSelector('TrackSelector', self.blackboard)
        pan_tilt = PanTiltPublisher('PanTiltPublisher', self.blackboard, self.cmd_pub)
        sequence = py_trees.composites.Sequence('MainSequence', memory=False)
        sequence.add_children([selector, pan_tilt])
        self.tree = py_trees_ros.trees.BehaviourTree(sequence)

    # -----------------------------------------------------------------------
    # FSM transition helpers
    # Each one is a no-op if the state is already correct, logs every real change,
    # and publishes a JSON trace to /skull_control/state_transition.
    # -----------------------------------------------------------------------

    def _transition_general(self, new_state: GeneralState, event: str):
        old_state = self.blackboard.general_state
        if old_state == new_state:
            return
        self.blackboard.general_state = new_state
        self.blackboard.last_state_change_ts = time.monotonic()
        self.get_logger().info(f'General: {old_state.value} -> {new_state.value} [{event}]')
        self._publish_transition('general', old_state.value, new_state.value, event)

    def _transition_eye(self, new_state: EyeState, event: str):
        old_state = self.blackboard.eye_state
        if old_state == new_state:
            return
        self.blackboard.eye_state = new_state
        self.get_logger().info(f'Eye: {old_state.value} -> {new_state.value} [{event}]')
        self._publish_transition('eye', old_state.value, new_state.value, event)

    def _transition_llm(self, new_state: LLMState, event: str):
        old_state = self.blackboard.llm_state
        if old_state == new_state:
            return
        self.blackboard.llm_state = new_state
        self.get_logger().info(f'LLM: {old_state.value} -> {new_state.value} [{event}]')
        self._publish_transition('llm', old_state.value, new_state.value, event)

    def _transition_speech(self, new_state: SpeechState, event: str):
        old_state = self.blackboard.speech_state
        if old_state == new_state:
            return
        self.blackboard.speech_state = new_state
        if new_state == SpeechState.SPEAKING:
            # Reset the stall watchdog from now, not from a stale previous timestamp
            self.blackboard.last_audio_chunk_ts = time.monotonic()
        self.get_logger().info(f'Speech: {old_state.value} -> {new_state.value} [{event}]')
        self._publish_transition('speech', old_state.value, new_state.value, event)

    def _publish_transition(self, subsystem: str, old_state: str, new_state: str, event: str):
        """Emit a JSON state-transition trace to /skull_control/state_transition."""
        msg = String()
        ts = time.time()
        msg.data = (
            f'{{"ts": {ts:.3f}, "subsystem": "{subsystem}", '
            f'"from": "{old_state}", "to": "{new_state}", "event": "{event}"}}'
        )
        self.state_transition_pub.publish(msg)

    def _publish_control(self, publisher, command: str):
        """Send a single-word control command string to a downstream node."""
        msg = String()
        msg.data = command
        publisher.publish(msg)

    # -----------------------------------------------------------------------
    # Eye overlay helpers
    # DISPLAY_EMOTION is a timed overlay; it reverts to the base eye state
    # after DISPLAY_EMOTION_TIMEOUT_SEC regardless of what else happens.
    # -----------------------------------------------------------------------

    def _set_eye_display_emotion_overlay(self, event: str):
        self._transition_eye(EyeState.DISPLAY_EMOTION, event)
        self.blackboard.eye_overlay_until_ts = time.monotonic() + DISPLAY_EMOTION_TIMEOUT_SEC

    def _overlay_active(self) -> bool:
        """True if the DISPLAY_EMOTION overlay is still within its timeout window."""
        return (
            self.blackboard.eye_state == EyeState.DISPLAY_EMOTION
            and time.monotonic() < self.blackboard.eye_overlay_until_ts
        )

    # -----------------------------------------------------------------------
    # Interrupt token detection
    # Only the *first* word of the transcript is checked; a bare "HALT" or
    # "HOLD" triggers an interrupt, but "please halt" does not.
    # -----------------------------------------------------------------------

    def _extract_interrupt_token(self, transcript: str):
        """Return the interrupt token if the transcript starts with one, else None."""
        normalized = re.sub(r'[^A-Z ]', ' ', transcript.upper())
        words = [w for w in normalized.split() if w]
        if words and words[0] in INTERRUPT_TOKENS:
            return words[0]
        return None

    # -----------------------------------------------------------------------
    # Echo suppression
    # Prevents the skull from hearing its own voice fed back through the mic.
    # Two layers: a hard time blackout, then a fuzzy text similarity check.
    # -----------------------------------------------------------------------

    def _normalize_for_echo_compare(self, text: str) -> str:
        """Lowercase, strip punctuation, collapse whitespace."""
        cleaned = re.sub(r'[^a-z0-9 ]', ' ', text.lower())
        return re.sub(r'\s+', ' ', cleaned).strip()

    def _looks_like_tts_echo(self, transcript: str, now: float) -> bool:
        """
        Compares incoming STT against the most recent TTS phrase using Jaccard
        token similarity. Returns True if it's probably mic bleed-through.
        """
        if not self.blackboard.last_tts_phrase:
            return False
        if (now - self.blackboard.last_tts_phrase_ts) > ECHO_TEXT_MATCH_WINDOW_SEC:
            return False

        a = self._normalize_for_echo_compare(transcript)
        b = self._normalize_for_echo_compare(self.blackboard.last_tts_phrase)
        if not a or not b:
            return False

        # Fast path: one string is a substring of the other (full/partial loopback)
        if (len(a) >= 8 and a in b) or (len(b) >= 8 and b in a):
            return True

        # Jaccard similarity on meaningful tokens (length >= 3)
        a_tokens = {tok for tok in a.split() if len(tok) >= 3}
        b_tokens = {tok for tok in b.split() if len(tok) >= 3}
        if not a_tokens or not b_tokens:
            return False

        similarity = len(a_tokens & b_tokens) / float(len(a_tokens | b_tokens))
        return similarity >= ECHO_TEXT_SIMILARITY_THRESHOLD

    # -----------------------------------------------------------------------
    # LLM input publishing
    # All messages to the LLM are wrapped in a structured JSON envelope so the
    # agent knows the source, urgency, and type of each input.
    # -----------------------------------------------------------------------

    def _publish_llm_envelope(
        self,
        *,
        channel: str,
        source: str,
        msg_type: str,
        urgency: str,
        event: str,
        text: str = '',
        payload=None,
    ):
        """Publish a structured JSON envelope to /skull_control/llm_input."""
        envelope = {
            'channel': channel,
            'source': source,
            'type': msg_type,
            'event': event,
            'urgency': urgency,
            'ts': time.time(),
        }
        if text:
            envelope['text'] = text
        if payload:
            envelope['payload'] = payload

        msg = String()
        msg.data = json.dumps(envelope, separators=(',', ':'))
        self.llm_input_pub.publish(msg)

    def _should_publish_system_event(self, event: str, payload, cooldown_sec: float) -> bool:
        """
        Rate-limiter for system events going to the LLM.
        Prevents spamming the LLM with the same event repeatedly.
        Two conditions must both pass:
          1. Minimum cooldown since last publish of this event type.
          2. Identical payload must have a wider suppression window.
        """
        now = time.monotonic()
        signature = json.dumps(payload or {}, sort_keys=True, separators=(',', ':'))
        last_ts = self.blackboard.last_system_event_publish_ts.get(event)
        last_sig = self.blackboard.last_system_event_signature.get(event)

        if last_ts is not None and (now - last_ts) < cooldown_sec:
            return False

        if (
            last_ts is not None
            and last_sig == signature
            and (now - last_ts) < IDENTICAL_EVENT_SUPPRESS_SEC
        ):
            return False

        self.blackboard.last_system_event_publish_ts[event] = now
        self.blackboard.last_system_event_signature[event] = signature
        return True

    def _publish_system_event_to_llm(
        self,
        *,
        source: str,
        event: str,
        urgency: str,
        payload=None,
        text: str = '',
    ):
        """Publish a system/FSM event to the LLM, subject to rate limiting."""
        cooldown = SYSTEM_EVENT_COOLDOWN_SEC.get(event, SYSTEM_EVENT_COOLDOWN_DEFAULT_SEC)
        if not self._should_publish_system_event(event, payload, cooldown):
            return

        self._publish_llm_envelope(
            channel='system',
            source=source,
            msg_type='event',
            event=event,
            urgency=urgency,
            text=text,
            payload=payload,
        )

    def _forward_transcript_to_llm(self, transcript: str):
        """Wrap a human speech transcript in a 'human' channel envelope and publish."""
        self._publish_llm_envelope(
            channel='human',
            source='stt',
            msg_type='transcript',
            event='human_transcript',
            urgency='medium',
            text=transcript,
        )

    # -----------------------------------------------------------------------
    # Interrupt handling: stop TTS/speaker and cancel LLM on HALT/HOLD
    # -----------------------------------------------------------------------

    def _stop_tts(self):
        """Send STOP to both TTS synthesis and the speaker playback node."""
        self._publish_control(self.tts_control_pub, 'STOP')
        self._publish_control(self.speaker_control_pub, 'STOP')
        self.get_logger().warn('Interrupt: STOP sent to TTS and speaker.')

    def _cancel_llm_reasoning(self):
        """Send CANCEL to the LLM agent to abort any in-flight inference."""
        self._publish_control(self.llm_control_pub, 'CANCEL')
        self.get_logger().warn('Interrupt: CANCEL sent to LLM agent.')

    # -----------------------------------------------------------------------
    # FSM event handlers
    # These are called from topic callbacks or the tick loop to drive state
    # changes in response to real events.
    # -----------------------------------------------------------------------

    def on_llm_response_ready(self, verbal: bool, has_nonverbal: bool):
        """
        Called when the LLM has a response ready (triggered by seeing a TTS
        phrase on /text_to_speech/text_input).

        If the response includes speech (verbal=True), transitions all sub-machines
        to SPEAKING. If it's only a non-verbal action, shows an emotion overlay
        and returns to TRACKING.
        """
        if self.blackboard.general_state != GeneralState.THINKING:
            return

        if verbal:
            self._transition_llm(LLMState.SPEAKING, 'llm_response_ready_verbal')
            self._transition_speech(SpeechState.SPEAKING, 'tts_started')
            self._transition_general(GeneralState.SPEAKING, 'llm_response_ready_verbal')
            return

        # Non-verbal response: show an emotion overlay and go back to tracking
        if has_nonverbal:
            self._set_eye_display_emotion_overlay('llm_response_ready_nonverbal')
        self._transition_llm(LLMState.IDLE, 'llm_response_ready_nonverbal')
        self._transition_general(GeneralState.TRACKING, 'llm_response_ready_nonverbal')

    def on_tts_done(self, event: str = 'tts_done'):
        """
        Called when TTS/speaker finishes the current utterance (last audio chunk
        received, or stall timeout triggered).

        Arms a short echo-suppression window then returns the FSM to TRACKING or
        IDLE depending on whether a person is still in frame.
        """
        # Suppress the skull hearing its own voice immediately after speaking
        self.blackboard.stt_echo_suppress_until_ts = time.monotonic() + POST_TTS_ECHO_SUPPRESS_SEC

        self._transition_speech(SpeechState.IDLE, event)
        self._transition_llm(LLMState.IDLE, event)

        if self.blackboard.current_target is not None:
            self._transition_general(GeneralState.TRACKING, event)
            if not self._overlay_active():
                self._transition_eye(EyeState.TRACKING, event)
        else:
            self._transition_general(GeneralState.IDLE, event)
            if not self._overlay_active():
                self._transition_eye(EyeState.IDLE, event)

        # Do not feed tts_done back into the LLM: it burns context budget and
        # can trigger low-value follow-up generations.

    # -----------------------------------------------------------------------
    # ROS topic callbacks
    # -----------------------------------------------------------------------

    def tracked_persons_callback(self, msg: TrackedPersons):
        """Receive updated person tracks; refresh activity timestamps and trigger IDLE->TRACKING."""
        self.blackboard.tracks = msg.tracks
        now = time.monotonic()
        if msg.tracks:
            self.blackboard.last_motion_ts = now
            self.blackboard.last_interest_ts = now
            # Transition IDLE -> TRACKING on first detection
            if self.blackboard.general_state == GeneralState.IDLE:
                self._transition_general(GeneralState.TRACKING, 'person_detected')
                if not self._overlay_active():
                    self._transition_eye(EyeState.TRACKING, 'person_detected')
                self._publish_system_event_to_llm(
                    source='person_tracking',
                    event='person_detected',
                    urgency='low',
                    payload={'track_count': len(msg.tracks)},
                )

    def stt_callback(self, msg: String):
        """
        Handle incoming speech transcript.

        Processing order:
          1. Ignore empty transcripts.
          2. Check for interrupt tokens first (always processed, even while SPEAKING).
          3. Apply echo suppression (time window + fuzzy text match).
          4. Gate out transcripts while skull is already speaking.
          5. Forward valid user input to LLM and transition to THINKING.
        """
        transcript = msg.data.strip()
        if not transcript:
            return

        interrupt_token = self._extract_interrupt_token(transcript)
        is_interrupt = interrupt_token is not None
        now = time.monotonic()

        # Echo suppression: discard mic bleed from our own speaker
        if not is_interrupt and now < self.blackboard.stt_echo_suppress_until_ts:
            self.get_logger().info('STT ignored: within post-TTS echo suppression window.')
            return
        if not is_interrupt and self._looks_like_tts_echo(transcript, now):
            self.get_logger().info('STT ignored: matches recent TTS phrase (echo suppression).')
            return

        if is_interrupt:
            self._handle_interrupt(interrupt_token)
            return

        # STT gate: don't process new user speech while already speaking
        if self.blackboard.speech_state == SpeechState.SPEAKING:
            return

        # Valid new user speech: update activity timestamps and forward to LLM
        self.blackboard.last_speech_ts = time.monotonic()
        self.blackboard.last_interest_ts = self.blackboard.last_speech_ts

        if self.blackboard.general_state in (GeneralState.TRACKING, GeneralState.IDLE):
            self._transition_general(GeneralState.THINKING, 'speech_detected')
            self._transition_llm(LLMState.THINKING, 'speech_detected')
            if not self._overlay_active():
                self._transition_eye(EyeState.THINKING, 'speech_detected')
            self._forward_transcript_to_llm(transcript)

    def _handle_interrupt(self, interrupt_token: str):
        """Stop all active speech and reasoning, then return to TRACKING or IDLE."""
        self._stop_tts()
        self._cancel_llm_reasoning()

        self._transition_speech(SpeechState.IDLE, 'interrupt_detected')
        self._transition_llm(LLMState.IDLE, 'interrupt_detected')

        if self.blackboard.current_target is not None:
            self._transition_general(GeneralState.TRACKING, 'interrupt_detected')
            if not self._overlay_active():
                self._transition_eye(EyeState.TRACKING, 'interrupt_detected')
        else:
            self._transition_general(GeneralState.IDLE, 'interrupt_detected')
            if not self._overlay_active():
                self._transition_eye(EyeState.IDLE, 'interrupt_detected')

        self.blackboard.last_interest_ts = time.monotonic()
        self._publish_system_event_to_llm(
            source='stt',
            event='interrupt_detected',
            urgency='high',
            payload={'token': interrupt_token},
        )

    def tts_text_input_callback(self, msg: String):
        """
        Spy on outgoing TTS phrases so we can arm echo suppression and
        trigger the THINKING -> SPEAKING FSM transition.
        """
        phrase = msg.data.strip()
        if phrase:
            self.blackboard.last_tts_phrase = phrase
            self.blackboard.last_tts_phrase_ts = time.monotonic()
            self.on_llm_response_ready(verbal=True, has_nonverbal=False)

    def speaker_audio_callback(self, msg: AudioData):
        """Track audio chunk arrival; end SPEAKING state when the last chunk arrives."""
        self.blackboard.last_audio_chunk_ts = time.monotonic()
        if msg.is_last_chunk and self.blackboard.speech_state == SpeechState.SPEAKING:
            self.on_tts_done()

    def llm_status_callback(self, msg: String):
        """
        Handle runtime/agent health notifications from llm_agent_http_node.

        If the LLM fails while we're in THINKING, force a safe recovery so
        the FSM does not remain wedged waiting for a TTS signal.
        """
        try:
            status_payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Ignoring malformed /llm_agent/status payload: {msg.data!r}')
            return

        status = str(status_payload.get('status', '')).strip().lower()
        reason = str(status_payload.get('reason', '')).strip().lower()

        non_fatal_statuses = {'runtime_restarted', 'cancelled'}
        fatal_statuses = {'restart_failed', 'runtime_error_output', 'parse_error', 'error'}

        if status in non_fatal_statuses:
            return

        if status not in fatal_statuses:
            self.get_logger().warning(f'Unknown llm status received: {status_payload}')
            return

        if self.blackboard.general_state != GeneralState.THINKING and self.blackboard.llm_state != LLMState.THINKING:
            return

        event = f'llm_failure_{status}'
        if reason:
            event = f'{event}_{reason}'

        self.get_logger().error(
            'LLM failure while THINKING; forcing FSM recovery. '
            f'status={status} reason={reason or "n/a"}'
        )

        self._transition_llm(LLMState.IDLE, event)
        self._transition_speech(SpeechState.IDLE, event)
        if self.blackboard.current_target is not None:
            self._transition_general(GeneralState.TRACKING, event)
            if not self._overlay_active():
                self._transition_eye(EyeState.TRACKING, event)
        else:
            self._transition_general(GeneralState.IDLE, event)
            if not self._overlay_active():
                self._transition_eye(EyeState.IDLE, event)

    def test_event_callback(self, msg: String):
        """
        Debug/test hook: force FSM into specific states via /skull_control/test_event.
        Only active when enable_test_events parameter is True.

        Supported commands: FORCE_IDLE, FORCE_TRACKING, FORCE_THINKING,
                            FORCE_SPEAKING, MARK_TTS_DONE
        """
        event = msg.data.strip().upper()

        if event == 'FORCE_IDLE':
            self._transition_speech(SpeechState.IDLE, 'test_event_force_idle')
            self._transition_llm(LLMState.IDLE, 'test_event_force_idle')
            self._transition_general(GeneralState.IDLE, 'test_event_force_idle')
            if not self._overlay_active():
                self._transition_eye(EyeState.IDLE, 'test_event_force_idle')

        elif event == 'FORCE_TRACKING':
            now = time.monotonic()
            # Reset activity clocks so timeout checks start fresh
            self.blackboard.last_motion_ts = now
            self.blackboard.last_speech_ts = now
            self.blackboard.last_interest_ts = now
            self._transition_general(GeneralState.TRACKING, 'test_event_force_tracking')
            if not self._overlay_active():
                self._transition_eye(EyeState.TRACKING, 'test_event_force_tracking')

        elif event == 'FORCE_THINKING':
            self._transition_general(GeneralState.THINKING, 'test_event_force_thinking')
            self._transition_llm(LLMState.THINKING, 'test_event_force_thinking')
            if not self._overlay_active():
                self._transition_eye(EyeState.THINKING, 'test_event_force_thinking')

        elif event == 'FORCE_SPEAKING':
            self._transition_general(GeneralState.SPEAKING, 'test_event_force_speaking')
            self._transition_llm(LLMState.SPEAKING, 'test_event_force_speaking')
            self._transition_speech(SpeechState.SPEAKING, 'test_event_force_speaking')

        elif event == 'MARK_TTS_DONE':
            self.on_tts_done()

    # -----------------------------------------------------------------------
    # Tick loop
    # Runs at 10 Hz; checks all time-based FSM transitions.
    # -----------------------------------------------------------------------

    def _tick_fsm(self):
        """
        Time-driven FSM checks. Called every tick (100ms).

        Handles:
          - DISPLAY_EMOTION overlay expiry
          - Low-interest (bored) timeout
          - No-motion and no-speech nudge events to LLM
          - TRACKING -> IDLE when person gone and both timeouts expire
          - SPEAKING stall watchdog
        """
        now = time.monotonic()
        has_target = self.blackboard.current_target is not None

        # Expire the DISPLAY_EMOTION eye overlay
        if (
            self.blackboard.eye_state == EyeState.DISPLAY_EMOTION
            and now >= self.blackboard.eye_overlay_until_ts
        ):
            self._transition_eye(
                EyeState.TRACKING if has_target else EyeState.IDLE,
                'display_emotion_timeout',
            )

        if self.blackboard.general_state == GeneralState.TRACKING:
            # Person is present but nothing interesting is happening
            if has_target and (now - self.blackboard.last_interest_ts) >= LOW_INTEREST_TIMEOUT_SEC:
                if self.blackboard.eye_state == EyeState.TRACKING:
                    self._transition_eye(EyeState.BORED, 'low_interest_timeout')
                    self._publish_system_event_to_llm(
                        source='fsm',
                        event='low_interest_timeout',
                        urgency='low',
                        payload={
                            'seconds_without_interest': round(now - self.blackboard.last_interest_ts, 1),
                        },
                    )

            no_motion = (now - self.blackboard.last_motion_ts) >= NO_MOTION_TIMEOUT_SEC
            no_speech = (now - self.blackboard.last_speech_ts) >= NO_SPEECH_TIMEOUT_SEC

            # Nudge the LLM with timeout context events (rate-limited internally)
            if no_motion:
                self._publish_system_event_to_llm(
                    source='fsm',
                    event='no_motion_timeout',
                    urgency='low',
                    payload={
                        'seconds_without_motion': round(now - self.blackboard.last_motion_ts, 1),
                        'has_target': has_target,
                    },
                )
            if no_speech:
                self._publish_system_event_to_llm(
                    source='fsm',
                    event='no_speech_timeout',
                    urgency='low',
                    payload={
                        'seconds_without_speech': round(now - self.blackboard.last_speech_ts, 1),
                        'has_target': has_target,
                    },
                )

            # Drop to IDLE only when person is fully gone AND both timeouts have expired
            if no_motion and no_speech and not has_target:
                self._transition_general(GeneralState.IDLE, 'no_speech_timeout+no_motion_timeout')
                if not self._overlay_active():
                    self._transition_eye(EyeState.IDLE, 'no_speech_timeout+no_motion_timeout')

        # If motion is detected while bored, snap back to TRACKING
        if self.blackboard.eye_state == EyeState.BORED and has_target:
            self._transition_eye(EyeState.TRACKING, 'motion_detected')

        # Safety net: if we're supposedly speaking but no audio has arrived, give up
        if self.blackboard.general_state == GeneralState.SPEAKING:
            if (now - self.blackboard.last_audio_chunk_ts) >= SPEAKING_STALL_TIMEOUT_SEC:
                self.get_logger().warning('SPEAKING stall detected; forcing speech completion.')
                self.on_tts_done('speaking_stall_timeout')

        # Safety net: if LLM stalled and no TTS start arrives, recover from THINKING
        if self.blackboard.general_state == GeneralState.THINKING:
            if (now - self.blackboard.last_state_change_ts) >= THINKING_STALL_TIMEOUT_SEC:
                self.get_logger().error('THINKING stall detected; forcing recovery from LLM wait state.')
                self._transition_llm(LLMState.IDLE, 'thinking_stall_timeout')
                self._transition_speech(SpeechState.IDLE, 'thinking_stall_timeout')
                if has_target:
                    self._transition_general(GeneralState.TRACKING, 'thinking_stall_timeout')
                    if not self._overlay_active():
                        self._transition_eye(EyeState.TRACKING, 'thinking_stall_timeout')
                else:
                    self._transition_general(GeneralState.IDLE, 'thinking_stall_timeout')
                    if not self._overlay_active():
                        self._transition_eye(EyeState.IDLE, 'thinking_stall_timeout')

    def _on_tick(self):
        """Called at 10 Hz; drive the FSM checks then tick the behavior tree."""
        self._tick_fsm()
        self.tree.tick()

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = SkullControlBTNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

