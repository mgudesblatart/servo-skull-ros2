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
    def __init__(self, output=None, outputs=None):
        self.output = output or '{"thoughts":"brief","tool_calls":[{"say_phrase":{"msg":"Short answer."}}]}'
        self.outputs = list(outputs or [])
        self.reset_calls = 0
        self.chat_requests = []

    def reset(self):
        self.reset_calls += 1
        return True

    def generate_chat(self, messages, max_output_tokens=None):
        self.chat_requests.append(messages)
        if self.outputs:
            return self.outputs.pop(0)
        return self.output


@unittest.skipIf(IMPORT_ERROR is not None, f"ROS dependencies unavailable: {IMPORT_ERROR}")
class TestLlmHttpContextBudget(unittest.TestCase):
    def _make_node(self, *, max_window_tokens=240, max_turns=1, reset_threshold_tokens=120, enable_llm_summary_refinement=False, http_outputs=None):
        node = LLMAgentHttpNode.__new__(LLMAgentHttpNode)
        node._history_lock = threading.Lock()
        node._max_window_tokens = max_window_tokens
        node._budget_warning_level = "ok"
        node._disable_thinking = True
        node._summary_refinement_disable_thinking = False
        node._summary_refinement_max_output_tokens = 384
        node._enable_llm_summary_refinement = enable_llm_summary_refinement
        node._conversation_buffer = ConversationBuffer(
            system_prompt="System prompt",
            max_turns=max_turns,
            max_window_tokens=max_window_tokens,
            reset_threshold_tokens=reset_threshold_tokens,
        )
        node.http_client = _HttpClientStub(outputs=http_outputs)
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

    def test_parse_input_envelope_normalizes_and_recovers_human_transcript(self):
        node = LLMAgentHttpNode.__new__(LLMAgentHttpNode)

        msg = SimpleNamespace(
            channel=" HUMAN ",
            source=" stt ",
            type=" Transcript ",
            event=" Human_Transcript ",
            urgency=" MEDIUM ",
            ts=1.0,
            text="GOOD EVENING",
            payload_json="{}",
        )

        parsed = node._parse_input_envelope(msg)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["channel"], "human")
        self.assertEqual(parsed["source"], "stt")
        self.assertEqual(parsed["type"], "transcript")
        self.assertEqual(parsed["event"], "human_transcript")
        self.assertEqual(parsed["urgency"], "medium")

        # Invalid channel is recovered for canonical STT human transcript envelopes.
        recovered_msg = SimpleNamespace(
            channel="voice",
            source="stt",
            type="transcript",
            event="human_transcript",
            urgency="medium",
            ts=2.0,
            text="HELLO",
            payload_json="{}",
        )
        recovered = node._parse_input_envelope(recovered_msg)
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered["channel"], "human")

    def test_build_user_content_raw_marks_direct_human_classification(self):
        envelope = {
            "channel": "human",
            "source": "stt",
            "type": "transcript",
            "event": "human_transcript",
            "urgency": "medium",
            "text": "GOOD EVENING",
            "payload": {},
        }

        content = LLMAgentHttpNode._build_user_content_raw(envelope)

        self.assertIn("Input Class: DIRECT_HUMAN", content)
        self.assertIn("User transcript: GOOD EVENING", content)
        self.assertIn("Respond as JSON only", content)

    def test_refine_summary_state_with_llm_returns_normalized_state(self):
        node = self._make_node(
            enable_llm_summary_refinement=True,
            http_outputs=[
                '{"user_preferences":["prefers concise responses"],'
                '"active_topics":["fertilizer types"],'
                '"open_loops":["What should I use for tomatoes?"],'
                '"assistant_commitments":["I can explain the main fertilizer categories."],'
                '"known_facts":["Tomatoes usually like balanced feed with enough potassium."]}'
            ],
        )

        refined = node._refine_summary_state_with_llm(
            prior_summary_state={},
            recent_turns=[("Explain fertilizer types briefly.", "{\"thoughts\":\"x\",\"tool_calls\":[]}")],
            heuristic_summary_state={
                "user_preferences": ["prefers concise responses"],
                "active_topics": ["fertilizer types"],
                "open_loops": [],
                "assistant_commitments": [],
                "known_facts": [],
            },
        )

        self.assertIsNotNone(refined)
        self.assertIn("fertilizer types", refined["active_topics"])
        self.assertEqual(len(node.status_pub.messages), 0)

    def test_refine_summary_state_with_llm_falls_back_on_invalid_json(self):
        node = self._make_node(enable_llm_summary_refinement=True, http_outputs=["not json"])

        refined = node._refine_summary_state_with_llm(
            prior_summary_state={},
            recent_turns=[],
            heuristic_summary_state={
                "user_preferences": ["prefers concise responses"],
                "active_topics": [],
                "open_loops": [],
                "assistant_commitments": [],
                "known_facts": [],
            },
        )

        self.assertIsNone(refined)
        self.assertEqual(node.status_pub.messages[-1]["reason"], "context_summary_refine_invalid")

    def test_prompt_callback_uses_llm_refined_summary_when_enabled(self):
        node = self._make_node(
            max_window_tokens=220,
            max_turns=1,
            reset_threshold_tokens=140,
            enable_llm_summary_refinement=True,
            http_outputs=[
                '{"user_preferences":["prefers concise responses"],'
                '"active_topics":["context exhaustion"],'
                '"open_loops":["Explain the current plan briefly."],'
                '"assistant_commitments":["I can explain the current plan."],'
                '"known_facts":["We summarize, then reset."]}',
                '{"thoughts":"brief","tool_calls":[{"say_phrase":{"msg":"Short answer."}}]}'
            ],
        )
        node._conversation_buffer.add_turn(
            "Tell me about context exhaustion in detail.",
            '{"thoughts":"analysis","tool_calls":[{"say_phrase":{"msg":"We summarize, then reset."}}]}',
        )
        node._conversation_buffer.add_turn(
            "What happens next?",
            '{"thoughts":"analysis","tool_calls":[{"say_phrase":{"msg":"I can explain the current plan."}}]}',
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

        history = node._conversation_buffer.get_history_for_prompt()
        self.assertIn("context exhaustion", history[1]["content"])
        self.assertEqual(node.http_client.reset_calls, 1)
        self.assertEqual(len(node.http_client.chat_requests), 2)
        self.assertEqual(node.tts_pub.messages[-1]["data"], "Short answer.")


if __name__ == "__main__":
    unittest.main()