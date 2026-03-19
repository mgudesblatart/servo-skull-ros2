import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

IMPORT_ERROR = None
try:
    from skull_control_node.llm_agent_axcl_node import LLMAgentAxclNode
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
        self.messages.append(msg.data)


class _RuntimeClientStub:
    def __init__(self, output='{"thoughts":"x","tool_calls":[],"final_output":"ok"}'):
        self.output = output
        self.generate_calls = 0
        self.close_calls = 0
        self.start_calls = 0

    def generate(self, _prompt):
        self.generate_calls += 1
        return self.output

    def close(self):
        self.close_calls += 1

    def start(self):
        self.start_calls += 1


@unittest.skipIf(IMPORT_ERROR is not None, f"ROS dependencies unavailable: {IMPORT_ERROR}")
class TestLlmCancelBehavior(unittest.TestCase):
    def _make_node(self):
        node = LLMAgentAxclNode.__new__(LLMAgentAxclNode)
        node.cancel_requested = threading.Event()
        node.request_in_flight = threading.Event()
        node.runtime_reset_lock = threading.RLock()
        node.runtime_client = _RuntimeClientStub()
        node.inline_system_prompt = False
        node.system_prompt = ""
        node.tts_pub = _PublisherStub()
        node.get_logger = lambda: _LoggerStub()
        return node

    def test_prompt_drops_output_if_cancel_already_set(self):
        node = self._make_node()
        node.cancel_requested.set()

        with patch("skull_control_node.llm_agent_axcl_node.parse_response") as parse_mock, patch(
            "skull_control_node.llm_agent_axcl_node.extract_say_phrase_calls"
        ) as extract_mock:
            parse_mock.return_value = {"thoughts": "x", "tool_calls": [], "final_output": "ok"}
            extract_mock.return_value = ["hello"]

            node.prompt_callback(SimpleNamespace(data="test prompt"))

        self.assertEqual(node.runtime_client.generate_calls, 1)
        self.assertEqual(node.tts_pub.messages, [])
        self.assertFalse(node.cancel_requested.is_set())

    def test_control_cancel_resets_runtime_and_clears_cancel_when_request_active(self):
        node = self._make_node()
        node.request_in_flight.set()

        node.control_callback(SimpleNamespace(data="CANCEL"))

        self.assertEqual(node.runtime_client.close_calls, 1)
        self.assertEqual(node.runtime_client.start_calls, 1)
        self.assertFalse(node.cancel_requested.is_set())

    def test_control_cancel_without_active_request_skips_runtime_reset(self):
        node = self._make_node()

        node.control_callback(SimpleNamespace(data="CANCEL"))

        self.assertEqual(node.runtime_client.close_calls, 0)
        self.assertEqual(node.runtime_client.start_calls, 0)
        self.assertFalse(node.cancel_requested.is_set())


if __name__ == "__main__":
    unittest.main()
