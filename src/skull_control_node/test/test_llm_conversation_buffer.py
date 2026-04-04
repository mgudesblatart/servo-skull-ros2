import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skull_control_node.llm_conversation_buffer import ConversationBuffer


class TestConversationBuffer(unittest.TestCase):
    def test_trims_to_recent_turns_and_keeps_summary(self):
        buffer = ConversationBuffer(system_prompt="System prompt", max_turns=2, max_window_tokens=1000)

        for index in range(3):
            buffer.add_turn(
                f"User turn {index}\n\nRespond as JSON only with keys: thoughts and tool_calls.",
                '{"thoughts":"thinking","tool_calls":[{"say_phrase":{"msg":"reply %d"}}]}' % index,
            )

        history = buffer.get_history_for_prompt()
        self.assertEqual(history[0]["role"], "system")
        self.assertEqual(history[1]["role"], "system")
        self.assertIn("reply 0", history[1]["content"])
        self.assertEqual(len([message for message in history if message["role"] == "user"]), 2)
        self.assertEqual(len([message for message in history if message["role"] == "assistant"]), 2)

    def test_small_window_forces_summary_compaction(self):
        buffer = ConversationBuffer(max_turns=1, max_window_tokens=90, summary_max_chars=180)

        buffer.add_turn(
            "Tell me a very long story about servo skull context management and memory pressure.",
            '{"thoughts":"long reasoning","tool_calls":[{"say_phrase":{"msg":"A long but compact answer."}}]}',
        )
        buffer.add_turn(
            "Add more detail about what should happen before context is exhausted.",
            '{"thoughts":"more reasoning","tool_calls":[{"say_phrase":{"msg":"Warn first, then reset cleanly."}}]}',
        )

        self.assertLess(buffer.get_remaining_budget(), buffer.max_window_tokens)
        self.assertTrue(buffer.summary_text)

    def test_generate_reset_summary_prefers_spoken_phrase(self):
        buffer = ConversationBuffer(max_turns=1, max_window_tokens=1000)
        buffer.add_turn(
            "What is your name?",
            '{"thoughts":"identity","tool_calls":[{"say_phrase":{"msg":"I am Servo Skull."}}]}',
        )
        buffer.add_turn(
            "What can you do?",
            '{"thoughts":"capabilities","tool_calls":[{"say_phrase":{"msg":"I can track faces and speak."}}]}',
        )

        summary = buffer.generate_reset_summary()
        self.assertIn("I am Servo Skull.", summary)
        self.assertIn("What can you do?", summary)

    def test_reset_with_summary_reseeds_and_clears_turns(self):
        buffer = ConversationBuffer(system_prompt="System prompt", max_turns=2, max_window_tokens=120)
        buffer.add_turn(
            "Who are you?",
            '{"thoughts":"identity","tool_calls":[{"say_phrase":{"msg":"I am Servo Skull."}}]}',
        )
        reset_summary = buffer.generate_reset_summary()

        buffer.reset_with_summary(reset_summary)

        history = buffer.get_history_for_prompt()
        self.assertEqual(len([message for message in history if message["role"] == "user"]), 0)
        self.assertEqual(len([message for message in history if message["role"] == "assistant"]), 0)
        self.assertEqual(history[0]["content"], "System prompt")
        self.assertIn("I am Servo Skull.", history[1]["content"])
        self.assertGreaterEqual(buffer.get_remaining_budget("Next question"), 0)


if __name__ == "__main__":
    unittest.main()