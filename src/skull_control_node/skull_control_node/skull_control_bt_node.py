#!/usr/bin/env python3
import json
import re
from enum import Enum
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from servo_skull_msgs.msg import AudioData, TrackedPersons, TrackedPerson
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time

SENSOR_MIN = 0
SENSOR_MAX = 255
ESP32_MIN = 0
ESP32_MAX = 500
TARGET_TIMEOUT_SEC = 5
LOW_INTEREST_TIMEOUT_SEC = 60.0
NO_MOTION_TIMEOUT_SEC = 30.0
NO_SPEECH_TIMEOUT_SEC = 30.0
DISPLAY_EMOTION_TIMEOUT_SEC = 2.5
SPEAKING_STALL_TIMEOUT_SEC = 8.0
POST_TTS_ECHO_SUPPRESS_SEC = 0.6
ECHO_TEXT_MATCH_WINDOW_SEC = 8.0
ECHO_TEXT_SIMILARITY_THRESHOLD = 0.55
INTERRUPT_TOKENS = {'HALT', 'HOLD'}
SYSTEM_EVENT_COOLDOWN_DEFAULT_SEC = 10.0
IDENTICAL_EVENT_SUPPRESS_SEC = 30.0
SYSTEM_EVENT_COOLDOWN_SEC = {
    'person_detected': 8.0,
    'no_motion_timeout': 10.0,
    'no_speech_timeout': 10.0,
    'low_interest_timeout': 12.0,
    'interrupt_detected': 5.0,
    'tts_done': 5.0,
}


class GeneralState(Enum):
    IDLE = 'IDLE'
    TRACKING = 'TRACKING'
    THINKING = 'THINKING'
    SPEAKING = 'SPEAKING'


class EyeState(Enum):
    IDLE = 'IDLE'
    TRACKING = 'TRACKING'
    THINKING = 'THINKING'
    DISPLAY_EMOTION = 'DISPLAY_EMOTION'
    BORED = 'BORED'


class LLMState(Enum):
    IDLE = 'IDLE'
    THINKING = 'THINKING'
    SPEAKING = 'SPEAKING'


class SpeechState(Enum):
    IDLE = 'IDLE'
    SPEAKING = 'SPEAKING'

class Blackboard:
    def __init__(self):
        self.current_target = None
        self.current_target_id = None
        self.target_lost_time = None
        self.tracks = []
        self.general_state = GeneralState.IDLE
        self.eye_state = EyeState.IDLE
        self.llm_state = LLMState.IDLE
        self.speech_state = SpeechState.IDLE
        now = time.monotonic()
        self.last_motion_ts = now
        self.last_speech_ts = now
        self.last_interest_ts = now
        self.last_state_change_ts = now
        self.eye_overlay_until_ts = 0.0
        self.last_audio_chunk_ts = now
        self.stt_echo_suppress_until_ts = 0.0
        self.last_tts_phrase = ''
        self.last_tts_phrase_ts = 0.0
        self.last_system_event_publish_ts = {}
        self.last_system_event_signature = {}

class TrackSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super().__init__(name)
        self.blackboard = blackboard

    def update(self):
        tracks = self.blackboard.tracks
        now = time.monotonic()
        if not tracks:
            if self.blackboard.current_target_id is not None and self.blackboard.target_lost_time is None:
                self.blackboard.target_lost_time = now
            elif self.blackboard.target_lost_time is not None and (now - self.blackboard.target_lost_time) > TARGET_TIMEOUT_SEC:
                self.blackboard.current_target = None
                self.blackboard.current_target_id = None
                self.blackboard.target_lost_time = None
            return py_trees.common.Status.FAILURE
        found = None
        for p in tracks:
            if p.person_id == self.blackboard.current_target_id:
                found = p
                break
        if found:
            self.blackboard.current_target = found
            self.blackboard.target_lost_time = None
        else:
            best = max(tracks, key=lambda p: p.box_confidence)
            self.blackboard.current_target = best
            self.blackboard.current_target_id = best.person_id
            self.blackboard.target_lost_time = None
        return py_trees.common.Status.SUCCESS

class PanTiltPublisher(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, publisher):
        super().__init__(name)
        self.blackboard = blackboard
        self.publisher = publisher

    def update(self):
        if self.blackboard.current_target:
            x_raw = int((self.blackboard.current_target.box_left + self.blackboard.current_target.box_right) / 2)
            y_raw = int((self.blackboard.current_target.box_top + self.blackboard.current_target.box_bottom) / 2)
            x = int((x_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            y = int((y_raw - SENSOR_MIN) * (ESP32_MAX - ESP32_MIN) / (SENSOR_MAX - SENSOR_MIN) + ESP32_MIN)
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            self.publisher.publish(pt)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class SkullControlBTNode(Node):
    def __init__(self):
        super().__init__('skull_control_bt_node')
        self.declare_parameter('enable_test_events', False)
        self.enable_test_events = bool(self.get_parameter('enable_test_events').value)
        self.blackboard = Blackboard()
        self.subscription = self.create_subscription(
            TrackedPersons,
            '/person_tracking/tracked_persons',
            self.tracked_persons_callback,
            10
        )
        self.stt_subscription = self.create_subscription(
            String,
            '/speech_to_text/transcript',
            self.stt_callback,
            10
        )
        self.test_event_subscription = None
        if self.enable_test_events:
            self.test_event_subscription = self.create_subscription(
                String,
                '/skull_control/test_event',
                self.test_event_callback,
                10
            )
        self.tts_text_input_subscription = self.create_subscription(
            String,
            '/text_to_speech/text_input',
            self.tts_text_input_callback,
            10
        )
        self.speaker_audio_subscription = self.create_subscription(
            AudioData,
            '/speaker/audio_output',
            self.speaker_audio_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Point, 'skull_control/pan_tilt_cmd', 10)
        self.state_transition_pub = self.create_publisher(String, '/skull_control/state_transition', 50)
        self.llm_input_pub = self.create_publisher(String, '/skull_control/llm_input', 10)
        self.tts_control_pub = self.create_publisher(String, '/tts_node/control', 10)
        self.speaker_control_pub = self.create_publisher(String, '/speaker_node/control', 10)
        self.llm_control_pub = self.create_publisher(String, '/llm_agent_axcl/control', 10)
        # Build behavior tree
        selector = TrackSelector('TrackSelector', self.blackboard)
        pan_tilt = PanTiltPublisher('PanTiltPublisher', self.blackboard, self.cmd_pub)
        sequence = py_trees.composites.Sequence('MainSequence', memory=False)
        sequence.add_children([selector, pan_tilt])
        self.tree = py_trees_ros.trees.BehaviourTree(sequence)
        self.tick_timer = self.create_timer(0.1, self._on_tick)
        if self.enable_test_events:
            self.get_logger().warning('Test event hook enabled on /skull_control/test_event')
        self.get_logger().info('Skull Control BT Node started.')

    def _transition_general(self, new_state: GeneralState, event: str):
        old_state = self.blackboard.general_state
        if old_state == new_state:
            return
        self.blackboard.general_state = new_state
        self.blackboard.last_state_change_ts = time.monotonic()
        self.get_logger().info(
            f'General transition: {old_state.value} -> {new_state.value} (event={event})'
        )
        self._publish_transition('general', old_state.value, new_state.value, event)

    def _transition_eye(self, new_state: EyeState, event: str):
        old_state = self.blackboard.eye_state
        if old_state == new_state:
            return
        self.blackboard.eye_state = new_state
        self.get_logger().info(
            f'Eye transition: {old_state.value} -> {new_state.value} (event={event})'
        )
        self._publish_transition('eye', old_state.value, new_state.value, event)

    def _transition_llm(self, new_state: LLMState, event: str):
        old_state = self.blackboard.llm_state
        if old_state == new_state:
            return
        self.blackboard.llm_state = new_state
        self.get_logger().info(
            f'LLM transition: {old_state.value} -> {new_state.value} (event={event})'
        )
        self._publish_transition('llm', old_state.value, new_state.value, event)

    def _transition_speech(self, new_state: SpeechState, event: str):
        old_state = self.blackboard.speech_state
        if old_state == new_state:
            return
        self.blackboard.speech_state = new_state
        if new_state == SpeechState.SPEAKING:
            # Arm stall watchdog from speech-start, not from potentially stale prior audio timestamp.
            self.blackboard.last_audio_chunk_ts = time.monotonic()
        self.get_logger().info(
            f'Speech transition: {old_state.value} -> {new_state.value} (event={event})'
        )
        self._publish_transition('speech', old_state.value, new_state.value, event)

    def _publish_transition(self, subsystem: str, old_state: str, new_state: str, event: str):
        msg = String()
        ts = time.time()
        msg.data = (
            f'{{"ts": {ts:.3f}, "subsystem": "{subsystem}", '
            f'"from": "{old_state}", "to": "{new_state}", "event": "{event}"}}'
        )
        self.state_transition_pub.publish(msg)

    def _publish_control(self, publisher, command: str):
        msg = String()
        msg.data = command
        publisher.publish(msg)

    def _set_eye_display_emotion_overlay(self, event: str):
        self._transition_eye(EyeState.DISPLAY_EMOTION, event)
        self.blackboard.eye_overlay_until_ts = time.monotonic() + DISPLAY_EMOTION_TIMEOUT_SEC

    def _overlay_active(self):
        return (
            self.blackboard.eye_state == EyeState.DISPLAY_EMOTION
            and time.monotonic() < self.blackboard.eye_overlay_until_ts
        )

    def _contains_interrupt_token(self, transcript: str):
        return self._extract_interrupt_token(transcript) is not None

    def _extract_interrupt_token(self, transcript: str):
        normalized = re.sub(r'[^A-Z ]', ' ', transcript.upper())
        words = [word for word in normalized.split() if word]
        if not words:
            return None
        # Accept explicit command forms only (e.g. "HALT" or "HALT now").
        if words[0] in INTERRUPT_TOKENS:
            return words[0]
        return None

    def _normalize_for_echo_compare(self, text: str):
        cleaned = re.sub(r'[^a-z0-9 ]', ' ', text.lower())
        cleaned = re.sub(r'\s+', ' ', cleaned).strip()
        return cleaned

    def _looks_like_tts_echo(self, transcript: str, now: float):
        if not self.blackboard.last_tts_phrase:
            return False
        if (now - self.blackboard.last_tts_phrase_ts) > ECHO_TEXT_MATCH_WINDOW_SEC:
            return False

        a = self._normalize_for_echo_compare(transcript)
        b = self._normalize_for_echo_compare(self.blackboard.last_tts_phrase)
        if not a or not b:
            return False

        # Fast path for obvious repeats (full/partial phrase capture from mic loopback).
        if (len(a) >= 8 and a in b) or (len(b) >= 8 and b in a):
            return True

        a_tokens = {tok for tok in a.split() if len(tok) >= 3}
        b_tokens = {tok for tok in b.split() if len(tok) >= 3}
        if not a_tokens or not b_tokens:
            return False

        similarity = len(a_tokens & b_tokens) / float(len(a_tokens | b_tokens))
        return similarity >= ECHO_TEXT_SIMILARITY_THRESHOLD

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

    def _should_publish_system_event(self, event: str, payload, cooldown_sec: float):
        now = time.monotonic()
        signature = json.dumps(payload or {}, sort_keys=True, separators=(',', ':'))
        last_ts = self.blackboard.last_system_event_publish_ts.get(event)
        last_sig = self.blackboard.last_system_event_signature.get(event)

        if last_ts is not None and (now - last_ts) < cooldown_sec:
            return False

        # Suppress repeated identical event payloads for a wider debounce window.
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

    def _stop_tts(self):
        self._publish_control(self.tts_control_pub, 'STOP')
        self._publish_control(self.speaker_control_pub, 'STOP')
        self.get_logger().warn('Interrupt requested: sent STOP to TTS and speaker.')

    def _cancel_llm_reasoning(self):
        self._publish_control(self.llm_control_pub, 'CANCEL')
        self.get_logger().warn('Interrupt requested: sent CANCEL to LLM agent.')

    def on_llm_response_ready(self, verbal: bool, has_nonverbal: bool):
        # Hook for LLM integration. Call this when the llm_agent returns a response decision.
        if self.blackboard.general_state != GeneralState.THINKING:
            return
        if verbal:
            self._transition_llm(LLMState.SPEAKING, 'llm_response_ready_verbal')
            self._transition_speech(SpeechState.SPEAKING, 'tts_started')
            self._transition_general(GeneralState.SPEAKING, 'llm_response_ready_verbal')
            return
        if has_nonverbal:
            self._set_eye_display_emotion_overlay('llm_response_ready_nonverbal')
        self._transition_llm(LLMState.IDLE, 'llm_response_ready_nonverbal')
        self._transition_general(GeneralState.TRACKING, 'llm_response_ready_nonverbal')

    def on_tts_done(self, event: str = 'tts_done'):
        # Block short-lived speaker bleed-through from re-entering STT after speech completion.
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
        self._publish_system_event_to_llm(
            source='fsm',
            event='tts_done',
            urgency='low',
            payload={
                'has_target': self.blackboard.current_target is not None,
                'general_state': self.blackboard.general_state.value,
            },
        )

    def _forward_transcript_to_llm(self, transcript: str):
        self._publish_llm_envelope(
            channel='human',
            source='stt',
            msg_type='transcript',
            event='human_transcript',
            urgency='medium',
            text=transcript,
        )

    def tts_text_input_callback(self, msg: String):
        phrase = msg.data.strip()
        if phrase:
            self.blackboard.last_tts_phrase = phrase
            self.blackboard.last_tts_phrase_ts = time.monotonic()
            self.on_llm_response_ready(verbal=True, has_nonverbal=False)

    def speaker_audio_callback(self, msg: AudioData):
        self.blackboard.last_audio_chunk_ts = time.monotonic()
        if msg.is_last_chunk and self.blackboard.speech_state == SpeechState.SPEAKING:
            self.on_tts_done()

    def test_event_callback(self, msg: String):
        event = msg.data.strip().upper()
        if event == 'FORCE_IDLE':
            self._transition_speech(SpeechState.IDLE, 'test_event_force_idle')
            self._transition_llm(LLMState.IDLE, 'test_event_force_idle')
            self._transition_general(GeneralState.IDLE, 'test_event_force_idle')
            if not self._overlay_active():
                self._transition_eye(EyeState.IDLE, 'test_event_force_idle')
            return
        if event == 'FORCE_TRACKING':
            now = time.monotonic()
            # Keep test forcing deterministic by resetting timeout clocks.
            self.blackboard.last_motion_ts = now
            self.blackboard.last_speech_ts = now
            self.blackboard.last_interest_ts = now
            self._transition_general(GeneralState.TRACKING, 'test_event_force_tracking')
            if not self._overlay_active():
                self._transition_eye(EyeState.TRACKING, 'test_event_force_tracking')
            return
        if event == 'FORCE_THINKING':
            self._transition_general(GeneralState.THINKING, 'test_event_force_thinking')
            self._transition_llm(LLMState.THINKING, 'test_event_force_thinking')
            if not self._overlay_active():
                self._transition_eye(EyeState.THINKING, 'test_event_force_thinking')
            return
        if event == 'FORCE_SPEAKING':
            self._transition_general(GeneralState.SPEAKING, 'test_event_force_speaking')
            self._transition_llm(LLMState.SPEAKING, 'test_event_force_speaking')
            self._transition_speech(SpeechState.SPEAKING, 'test_event_force_speaking')
            return
        if event == 'MARK_TTS_DONE':
            self.on_tts_done()

    def tracked_persons_callback(self, msg: TrackedPersons):
        self.blackboard.tracks = msg.tracks
        now = time.monotonic()
        if msg.tracks:
            self.blackboard.last_motion_ts = now
            self.blackboard.last_interest_ts = now
            if self.blackboard.general_state == GeneralState.IDLE:
                self._transition_general(GeneralState.TRACKING, 'person_detected')
                if not self._overlay_active():
                    self._transition_eye(EyeState.TRACKING, 'person_detected')
                self._publish_system_event_to_llm(
                    source='person_tracking',
                    event='person_detected',
                    urgency='low',
                    payload={
                        'track_count': len(msg.tracks),
                    },
                )

    def stt_callback(self, msg: String):
        transcript = msg.data.strip()
        if not transcript:
            return
        interrupt_token = self._extract_interrupt_token(transcript)
        is_interrupt = interrupt_token is not None
        now = time.monotonic()
        if not is_interrupt and now < self.blackboard.stt_echo_suppress_until_ts:
            self.get_logger().info('Ignoring STT transcript during post-TTS echo suppression window.')
            return
        if not is_interrupt and self._looks_like_tts_echo(transcript, now):
            self.get_logger().info('Ignoring STT transcript that matches recent TTS phrase (echo suppression).')
            return
        if is_interrupt:
            self._stop_tts()
            self._cancel_llm_reasoning()
            self._transition_speech(SpeechState.IDLE, 'interrupt_detected')
            self._transition_llm(LLMState.IDLE, 'interrupt_detected')
            if self.blackboard.current_target is not None:
                self._transition_general(GeneralState.TRACKING, 'interrupt_detected')
            else:
                self._transition_general(GeneralState.IDLE, 'interrupt_detected')
            if not self._overlay_active():
                if self.blackboard.current_target is not None:
                    self._transition_eye(EyeState.TRACKING, 'interrupt_detected')
                else:
                    self._transition_eye(EyeState.IDLE, 'interrupt_detected')
            self.blackboard.last_interest_ts = time.monotonic()
            self._publish_system_event_to_llm(
                source='stt',
                event='interrupt_detected',
                urgency='high',
                payload={
                    'token': interrupt_token,
                },
            )
            return
        if self.blackboard.speech_state == SpeechState.SPEAKING:
            return
        self.blackboard.last_speech_ts = time.monotonic()
        self.blackboard.last_interest_ts = self.blackboard.last_speech_ts
        if self.blackboard.general_state in (GeneralState.TRACKING, GeneralState.IDLE):
            self._transition_general(GeneralState.THINKING, 'speech_detected')
            self._transition_llm(LLMState.THINKING, 'speech_detected')
            if not self._overlay_active():
                self._transition_eye(EyeState.THINKING, 'speech_detected')
            self._forward_transcript_to_llm(transcript)

    def _tick_fsm(self):
        now = time.monotonic()
        has_target = self.blackboard.current_target is not None

        if (
            self.blackboard.eye_state == EyeState.DISPLAY_EMOTION
            and now >= self.blackboard.eye_overlay_until_ts
        ):
            if has_target:
                self._transition_eye(EyeState.TRACKING, 'display_emotion_timeout')
            else:
                self._transition_eye(EyeState.IDLE, 'display_emotion_timeout')

        if self.blackboard.general_state == GeneralState.TRACKING:
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
            if no_motion and no_speech and not has_target:
                self._transition_general(GeneralState.IDLE, 'no_speech_timeout+no_motion_timeout')
                if not self._overlay_active():
                    self._transition_eye(EyeState.IDLE, 'no_speech_timeout+no_motion_timeout')

        if self.blackboard.eye_state == EyeState.BORED and has_target:
            self._transition_eye(EyeState.TRACKING, 'motion_detected')

        if self.blackboard.general_state == GeneralState.SPEAKING:
            if (now - self.blackboard.last_audio_chunk_ts) >= SPEAKING_STALL_TIMEOUT_SEC:
                self.get_logger().warning('No audio chunks observed while SPEAKING; forcing speech completion.')
                self.on_tts_done('speaking_stall_timeout')

    def _on_tick(self):
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
