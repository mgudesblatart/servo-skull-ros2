import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

IMPORT_ERROR = None
try:
    from skull_control_node.llm_agent_http_node import LLMAgentHttpNode
    from skull_control_node.llm_conversation_buffer import ConversationBuffer
except ModuleNotFoundError as exc:
    IMPORT_ERROR = exc


class _LoggerStub:
    def debug(self, _msg):
        pass

    def info(self, _msg):
        pass

    def warning(self, _msg):
        pass

    def error(self, _msg):
        pass


class _PublisherStub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        payload = {}
        for attr in ("status", "reason", "detail", "data"):
            if hasattr(msg, attr):
                payload[attr] = getattr(msg, attr)
        self.messages.append(payload)


class _HttpClientStub:
    def __init__(self, output=None):
        self.output = output or '{"thoughts":"brief","tool_calls":[{"say_phrase":{"msg":"Short answer."}}]}'
        self.reset_calls = 0
        self.chat_requests = []

    def reset(self):
        self.reset_calls += 1
        return True

    def generate_chat(self, messages):
        self.chat_requests.append(messages)
        return self.output


@unittest.skipIf(IMPORT_ERROR is not None, f"ROS dependencies unavailable: {IMPORT_ERROR}")
class TestLlmHttpContextBudget(unittest.TestCase):
    def _make_node(self, *, max_window_tokens=240, max_turns=1, reset_threshold_tokens=120):
        node = LLMAgentHttpNode.__new__(LLMAgentHttpNode)
        node._history_lock = threading.Lock()
        node._max_window_tokens = max_window_tokens
        node._budget_warning_level = "ok"
        node._conversation_buffer = ConversationBuffer(
            system_prompt="System prompt",
            max_turns=max_turns,
            max_window_tokens=max_window_tokens,
            reset_threshold_tokens=reset_threshold_tokens,
        )
        node.http_client = _HttpClientStub()
        node.status_pub = _PublisherStub()
        node.tts_pub = _PublisherStub()
        node.cancel_requested = threading.Event()
        node.request_in_flight = threading.Event()
        node._server_ready = True
        node.get_logger = lambda: _LoggerStub()
        return node

    def test_publish_budget_status_emits_low_then_critical_once(self):
        node = self._make_node(max_window_tokens=1000, reset_threshold_tokens=100)

        node._publish_budget_status(180)
        node._publish_budget_status(150)
        node._publish_budget_status(50)

        self.assertEqual(
            [msg["reason"] for msg in node.status_pub.messages],
            ["context_budget_low", "context_budget_critical"],
        )

    def test_perform_context_reset_reseeds_summary_and_clears_turns(self):
        node = self._make_node(max_window_tokens=220, max_turns=1, reset_threshold_tokens=140)
        node._conversation_buffer.add_turn(
            "What is your name?",
            '{"thoughts":"identity","tool_calls":[{"say_phrase":{"msg":"I am Servo Skull."}}]}',
        )
        node._conversation_buffer.add_turn(
            "What can you do?",
            '{"thoughts":"capabilities","tool_calls":[{"say_phrase":{"msg":"I can track faces and speak."}}]}',
        )

        node._perform_context_reset("Tell me more about your capabilities.")

        history = node._conversation_buffer.get_history_for_prompt()
        self.assertEqual(len([msg for msg in history if msg["role"] == "user"]), 0)
        self.assertEqual(len([msg for msg in history if msg["role"] == "assistant"]), 0)
        self.assertIn("Conversation summary:", history[1]["content"])
        self.assertIn("I am Servo Skull.", history[1]["content"])
        self.assertEqual(node.http_client.reset_calls, 1)
        self.assertEqual(
            [msg["reason"] for msg in node.status_pub.messages[:2]],
            ["context_reset_requested", "context_budget_reset"],
        )

    def test_prompt_callback_resets_then_generates_response(self):
        node = self._make_node(max_window_tokens=220, max_turns=1, reset_threshold_tokens=140)
        node._conversation_buffer.add_turn(
            "Tell me about servo skull memory pressure and how context exhaustion works in detail.",
            '{"thoughts":"analysis","tool_calls":[{"say_phrase":{"msg":"We need to manage context carefully."}}]}',
        )
        node._conversation_buffer.add_turn(
            "What happens next?",
            '{"thoughts":"more analysis","tool_calls":[{"say_phrase":{"msg":"We summarize, then reset."}}]}',
        )

        msg = SimpleNamespace(
            channel="human",
            source="stt",
            type="transcript",
            event="human_transcript",
            urgency="medium",
            ts=1.0,
            text="Explain the current plan briefly.",
            payload_json="{}",
        )

        node.prompt_callback(msg)

        reasons = [entry.get("reason") for entry in node.status_pub.messages if entry.get("reason")]
        self.assertIn("context_reset_requested", reasons)
        self.assertIn("context_budget_reset", reasons)
        self.assertEqual(node.http_client.reset_calls, 1)
        self.assertEqual(node.tts_pub.messages[-1]["data"], "Short answer.")
        self.assertTrue(node.http_client.chat_requests)


if __name__ == "__main__":
    unittest.main()