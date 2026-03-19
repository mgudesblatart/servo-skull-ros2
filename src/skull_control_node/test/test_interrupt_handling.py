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

    def _make_node(self, has_target: bool):
        node = SkullControlBTNode.__new__(SkullControlBTNode)
        node.blackboard = Blackboard()
        node.blackboard.general_state = GeneralState.THINKING
        node.blackboard.llm_state = LLMState.THINKING
        node.blackboard.speech_state = SpeechState.IDLE
        node.blackboard.eye_state = EyeState.THINKING
        node.blackboard.current_target = object() if has_target else None

        node._overlay_active = lambda: False
        node._stop_tts_called = 0
        node._cancel_llm_called = 0
        node.forwarded = []
        node.get_logger = lambda: self._LoggerStub()
        node._publish_transition = lambda *_args, **_kwargs: None

        node._stop_tts = lambda: setattr(node, "_stop_tts_called", node._stop_tts_called + 1)
        node._cancel_llm_reasoning = lambda: setattr(node, "_cancel_llm_called", node._cancel_llm_called + 1)
        node._forward_transcript_to_llm = lambda text: node.forwarded.append(text)

        return node

    def test_interrupt_in_thinking_with_target_returns_to_tracking(self):
        node = self._make_node(has_target=True)
        before = node.blackboard.last_interest_ts

        node.stt_callback(SimpleNamespace(data="HALT now"))

        self.assertEqual(node._stop_tts_called, 1)
        self.assertEqual(node._cancel_llm_called, 1)
        self.assertEqual(node.blackboard.general_state, GeneralState.TRACKING)
        self.assertEqual(node.blackboard.llm_state, LLMState.IDLE)
        self.assertEqual(node.blackboard.speech_state, SpeechState.IDLE)
        self.assertEqual(node.blackboard.eye_state, EyeState.TRACKING)
        self.assertEqual(node.forwarded, [])
        self.assertGreaterEqual(node.blackboard.last_interest_ts, before)

    def test_interrupt_in_thinking_without_target_returns_to_idle(self):
        node = self._make_node(has_target=False)

        node.stt_callback(SimpleNamespace(data="HOLD"))

        self.assertEqual(node._stop_tts_called, 1)
        self.assertEqual(node._cancel_llm_called, 1)
        self.assertEqual(node.blackboard.general_state, GeneralState.IDLE)
        self.assertEqual(node.blackboard.eye_state, EyeState.IDLE)
        self.assertEqual(node.forwarded, [])

    def test_non_interrupt_is_blocked_while_speaking(self):
        node = self._make_node(has_target=True)
        node.blackboard.general_state = GeneralState.SPEAKING
        node.blackboard.speech_state = SpeechState.SPEAKING

        node.stt_callback(SimpleNamespace(data="normal transcript"))

        self.assertEqual(node._stop_tts_called, 0)
        self.assertEqual(node._cancel_llm_called, 0)
        self.assertEqual(node.forwarded, [])


if __name__ == "__main__":
    unittest.main()
