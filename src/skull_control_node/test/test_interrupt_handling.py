import unittest
from types import SimpleNamespace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

IMPORT_ERROR = None
try:
    from skull_control_node.skull_control_bt_node import (
        Blackboard,
        EyeState,
        GeneralState,
        LLMState,
        SkullControlBTNode,
        SpeechState,
    )
except ModuleNotFoundError as exc:
    IMPORT_ERROR = exc


@unittest.skipIf(IMPORT_ERROR is not None, f"ROS dependencies unavailable: {IMPORT_ERROR}")
class TestInterruptHandling(unittest.TestCase):
    class _LoggerStub:
        def info(self, _msg):
            pass

        def warn(self, _msg):
            pass

        def warning(self, _msg):
            pass

        def error(self, _msg):
            pass

    def _make_node(self, has_target: bool):
        node = SkullControlBTNode.__new__(SkullControlBTNode)
        node.blackboard = Blackboard()
        node.blackboard.general_state = GeneralState.THINKING
        node.blackboard.llm_state = LLMState.THINKING
        node.blackboard.speech_state = SpeechState.IDLE
        node.blackboard.eye_state = EyeState.THINKING
        node.blackboard.current_target = object() if has_target else None

        node._overlay_active = lambda: False
        node.forwarded = []
        node.get_logger = lambda: self._LoggerStub()
        node._publish_transition = lambda *_args, **_kwargs: None

        node._forward_transcript_to_llm = lambda text: node.forwarded.append(text)

        return node

    def test_interrupt_token_is_treated_as_normal_transcript_in_tracking(self):
        node = self._make_node(has_target=True)
        node.blackboard.general_state = GeneralState.TRACKING
        node.blackboard.llm_state = LLMState.IDLE
        node.blackboard.speech_state = SpeechState.IDLE
        node.blackboard.eye_state = EyeState.TRACKING

        node.stt_callback(SimpleNamespace(data="HALT now"))

        self.assertEqual(node.blackboard.general_state, GeneralState.THINKING)
        self.assertEqual(node.blackboard.llm_state, LLMState.THINKING)
        self.assertEqual(node.blackboard.eye_state, EyeState.THINKING)
        self.assertEqual(node.forwarded, ["HALT now"])

    def test_interrupt_token_is_treated_as_normal_transcript_in_idle(self):
        node = self._make_node(has_target=False)
        node.blackboard.general_state = GeneralState.IDLE
        node.blackboard.llm_state = LLMState.IDLE
        node.blackboard.speech_state = SpeechState.IDLE
        node.blackboard.eye_state = EyeState.IDLE

        node.stt_callback(SimpleNamespace(data="RESET"))

        self.assertEqual(node.blackboard.general_state, GeneralState.THINKING)
        self.assertEqual(node.blackboard.llm_state, LLMState.THINKING)
        self.assertEqual(node.blackboard.eye_state, EyeState.THINKING)
        self.assertEqual(node.forwarded, ["RESET"])

    def test_non_interrupt_is_blocked_while_speaking(self):
        node = self._make_node(has_target=True)
        node.blackboard.general_state = GeneralState.SPEAKING
        node.blackboard.speech_state = SpeechState.SPEAKING

        node.stt_callback(SimpleNamespace(data="normal transcript"))

        self.assertEqual(node.forwarded, [])

    def test_last_chunk_does_not_finish_speaking_when_playback_timing_present(self):
        node = self._make_node(has_target=True)
        node.blackboard.general_state = GeneralState.SPEAKING
        node.blackboard.llm_state = LLMState.SPEAKING
        node.blackboard.speech_state = SpeechState.SPEAKING
        node.blackboard.awaiting_speaker_done = True
        node.blackboard.playback_expected_end_ts = 9999999.0

        node.speaker_audio_callback(SimpleNamespace(is_last_chunk=True))

        self.assertEqual(node.blackboard.general_state, GeneralState.SPEAKING)
        self.assertEqual(node.blackboard.speech_state, SpeechState.SPEAKING)
        self.assertEqual(node.blackboard.llm_state, LLMState.SPEAKING)

    def test_playback_done_event_finishes_speaking(self):
        node = self._make_node(has_target=False)
        node.blackboard.general_state = GeneralState.SPEAKING
        node.blackboard.llm_state = LLMState.SPEAKING
        node.blackboard.speech_state = SpeechState.SPEAKING
        node.blackboard.eye_state = EyeState.THINKING
        node.blackboard.awaiting_speaker_done = True

        node.speaker_playback_done_callback(SimpleNamespace(data=123.456))

        self.assertEqual(node.blackboard.general_state, GeneralState.IDLE)
        self.assertEqual(node.blackboard.llm_state, LLMState.IDLE)
        self.assertEqual(node.blackboard.speech_state, SpeechState.IDLE)
        self.assertEqual(node.blackboard.eye_state, EyeState.IDLE)

    def test_tick_finishes_speaking_when_playback_expected_end_reached(self):
        node = self._make_node(has_target=False)
        node.blackboard.general_state = GeneralState.SPEAKING
        node.blackboard.llm_state = LLMState.SPEAKING
        node.blackboard.speech_state = SpeechState.SPEAKING
        node.blackboard.eye_state = EyeState.THINKING
        node.blackboard.playback_expected_end_ts = 0.001

        node._tick_fsm()

        self.assertEqual(node.blackboard.general_state, GeneralState.IDLE)
        self.assertEqual(node.blackboard.llm_state, LLMState.IDLE)
        self.assertEqual(node.blackboard.speech_state, SpeechState.IDLE)
        self.assertEqual(node.blackboard.eye_state, EyeState.IDLE)


if __name__ == "__main__":
    unittest.main()
