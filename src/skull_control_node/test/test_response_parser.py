import unittest

from skull_control_node.response_parser import extract_say_phrase_calls, parse_response


class TestResponseParser(unittest.TestCase):
    def test_parse_plain_json(self):
        raw = '{"thoughts":"x","tool_calls":[],"final_output":"ok"}'
        parsed = parse_response(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["final_output"], "ok")

    def test_parse_json_inside_logs(self):
        raw = 'noise\n[I] runtime stuff\n{"thoughts":"x","tool_calls":[{"say_phrase":{"msg":"hello"}}],"final_output":"Phrase said."}\nprompt >>'
        parsed = parse_response(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(extract_say_phrase_calls(parsed), ["hello"])

    def test_parse_markdown_fenced_json(self):
        raw = '```json\n{"thoughts":"x","tool_calls":[],"final_output":"ok"}\n```'
        parsed = parse_response(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["final_output"], "ok")

    def test_parse_non_json_text_fallback_contract(self):
        raw = (
            "[I][ Run ][1016]: ttft: 488.55 ms\n"
            "Good evening! Is there anything you would like to talk about or ask about?\n"
            "[N][ Run ][1168]: hit eos,avg 9.09 token/s\n"
        )
        parsed = parse_response(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(
            parsed["final_output"],
            "Good evening! Is there anything you would like to talk about or ask about?",
        )
        self.assertEqual(
            extract_say_phrase_calls(parsed),
            ["Good evening! Is there anything you would like to talk about or ask about?"],
        )


if __name__ == "__main__":
    unittest.main()