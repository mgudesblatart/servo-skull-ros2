import threading
import unittest
from types import SimpleNamespace

from skull_control_node.utils.generation_stage import run_generation
from skull_control_node.utils.prompt_intake_stage import prepare_prompt_intake
from skull_control_node.utils.response_output_stage import persist_and_publish_response
from skull_control_node.utils.response_processing_stage import (
    ProcessedResponseArtifacts,
    process_model_response,
)


class _LoggerStub:
    def __init__(self):
        self.debugs = []
        self.infos = []
        self.warnings = []
        self.errors = []

    def debug(self, msg):
        self.debugs.append(msg)

    def info(self, msg):
        self.infos.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class TestPromptIntakeStage(unittest.TestCase):
    def _msg(self, **overrides):
        defaults = {
            "channel": "human",
            "source": "stt",
            "type": "transcript",
            "event": "human_transcript",
            "urgency": "medium",
            "ts": 1.0,
            "text": "Tell me about Orcs",
            "payload_json": "{}",
        }
        defaults.update(overrides)
        return SimpleNamespace(**defaults)

    def test_prepare_prompt_intake_rejects_empty_input(self):
        result = prepare_prompt_intake(
            self._msg(channel="", text=""),
            server_ready=True,
            valid_channels={"human", "system", "mixed"},
            valid_urgencies={"low", "medium", "high"},
            json_contract_hint="hint",
            default_disable_thinking=True,
            no_think_excerpt="/no_think",
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "empty_input")

    def test_prepare_prompt_intake_rejects_when_server_not_ready(self):
        result = prepare_prompt_intake(
            self._msg(),
            server_ready=False,
            valid_channels={"human", "system", "mixed"},
            valid_urgencies={"low", "medium", "high"},
            json_contract_hint="hint",
            default_disable_thinking=True,
            no_think_excerpt="/no_think",
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "server_not_ready")

    def test_prepare_prompt_intake_rejects_invalid_envelope(self):
        result = prepare_prompt_intake(
            self._msg(urgency="urgent"),
            server_ready=True,
            valid_channels={"human", "system", "mixed"},
            valid_urgencies={"low", "medium", "high"},
            json_contract_hint="hint",
            default_disable_thinking=True,
            no_think_excerpt="/no_think",
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "invalid_envelope")

    def test_prepare_prompt_intake_rejects_non_minimal_mode_lane(self):
        result = prepare_prompt_intake(
            self._msg(channel="system", source="fsm", type="event", event="person_detected"),
            server_ready=True,
            valid_channels={"human", "system", "mixed"},
            valid_urgencies={"low", "medium", "high"},
            json_contract_hint="hint",
            default_disable_thinking=True,
            no_think_excerpt="/no_think",
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "minimal_mode_drop")

    def test_prepare_prompt_intake_accepts_human_transcript_and_applies_no_think(self):
        result = prepare_prompt_intake(
            self._msg(),
            server_ready=True,
            valid_channels={"human", "system", "mixed"},
            valid_urgencies={"low", "medium", "high"},
            json_contract_hint="Respond as JSON only",
            default_disable_thinking=True,
            no_think_excerpt="/no_think",
        )
        self.assertTrue(result.accepted)
        self.assertIn("Input Class: DIRECT_HUMAN", result.user_content)
        self.assertIn("/no_think", result.user_content)


class TestGenerationStage(unittest.TestCase):
    def test_run_generation_success(self):
        logger = _LoggerStub()
        statuses = []
        request_in_flight = threading.Event()
        cancel_requested = threading.Event()

        result = run_generation(
            user_content="hello",
            build_messages=lambda user: [{"role": "user", "content": user}],
            http_generate_chat=lambda messages: '{"thoughts":"x","tool_calls":[]}',
            request_in_flight_event=request_in_flight,
            cancel_requested_event=cancel_requested,
            logger=logger,
            publish_status=lambda *args, **kwargs: statuses.append((args, kwargs)),
        )

        self.assertTrue(result.ok)
        self.assertEqual(result.reason, "")
        self.assertFalse(request_in_flight.is_set())
        self.assertEqual(statuses, [])

    def test_run_generation_error_path(self):
        logger = _LoggerStub()
        statuses = []

        def _raise(_messages):
            raise RuntimeError("boom")

        result = run_generation(
            user_content="hello",
            build_messages=lambda user: [{"role": "user", "content": user}],
            http_generate_chat=_raise,
            request_in_flight_event=threading.Event(),
            cancel_requested_event=threading.Event(),
            logger=logger,
            publish_status=lambda *args, **kwargs: statuses.append((args, kwargs)),
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "error")
        self.assertTrue(any(k.get("reason") == "http_inference_error" for _, k in statuses))

    def test_run_generation_cancelled_after_response(self):
        logger = _LoggerStub()
        statuses = []
        cancel_requested = threading.Event()

        def _http_generate(_messages):
            cancel_requested.set()
            return '{"thoughts":"x","tool_calls":[]}'

        result = run_generation(
            user_content="hello",
            build_messages=lambda user: [{"role": "user", "content": user}],
            http_generate_chat=_http_generate,
            request_in_flight_event=threading.Event(),
            cancel_requested_event=cancel_requested,
            logger=logger,
            publish_status=lambda *args, **kwargs: statuses.append((args, kwargs)),
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "cancelled_after_response")
        self.assertEqual(statuses, [])


class TestResponseProcessingStage(unittest.TestCase):
    def test_process_model_response_parse_failure_publishes_status(self):
        logger = _LoggerStub()
        statuses = []

        artifacts = process_model_response(
            raw_response="not-json",
            envelope={"channel": "human", "event": "human_transcript", "text": "hello"},
            user_content="hello",
            parse_response_fn=lambda _raw: None,
            extract_say_phrase_calls_fn=lambda _parsed: [],
            sanitize_spoken_phrase_fn=lambda phrase, **kwargs: phrase,
            build_history_user_content_fn=lambda envelope, user_content, compact_detail_fn=None: user_content,
            compact_detail_fn=lambda text, max_chars=180: text,
            normalize_assistant_history_contract_fn=lambda parsed, phrases, **kwargs: "",
            boilerplate_patterns=(),
            blocked_speech_markers=(),
            logger=logger,
            publish_status=lambda *args, **kwargs: statuses.append((args, kwargs)),
        )

        self.assertIsNone(artifacts)
        self.assertTrue(any(k.get("reason") == "invalid_model_output" for _, k in statuses))

    def test_process_model_response_success_builds_artifacts(self):
        parsed_contract = {
            "thoughts": "x",
            "tool_calls": [{"say_phrase": {"msg": "FOR THE EMPEROR"}}],
        }

        artifacts = process_model_response(
            raw_response='{"thoughts":"x","tool_calls":[{"say_phrase":{"msg":"FOR THE EMPEROR"}}]}',
            envelope={"channel": "human", "event": "human_transcript", "text": "hello"},
            user_content="hello",
            parse_response_fn=lambda _raw: parsed_contract,
            extract_say_phrase_calls_fn=lambda parsed: ["FOR THE EMPEROR", ""],
            sanitize_spoken_phrase_fn=lambda phrase, **kwargs: phrase.lower().strip(),
            build_history_user_content_fn=lambda envelope, user_content, compact_detail_fn=None: "history:user",
            compact_detail_fn=lambda text, max_chars=180: text,
            normalize_assistant_history_contract_fn=lambda parsed, phrases, **kwargs: "compact:assistant",
            boilerplate_patterns=("dummy",),
            blocked_speech_markers=("dummy",),
            logger=_LoggerStub(),
            publish_status=lambda *args, **kwargs: None,
        )

        self.assertIsNotNone(artifacts)
        self.assertEqual(artifacts.history_user_content, "history:user")
        self.assertEqual(artifacts.compact_assistant_text, "compact:assistant")
        self.assertEqual(artifacts.raw_assistant_text.startswith("{"), True)
        self.assertEqual(artifacts.phrases, ["for the emperor"])


class TestResponseOutputStage(unittest.TestCase):
    def test_persist_and_publish_response_appends_and_publishes(self):
        appended = []
        published = []

        artifacts = ProcessedResponseArtifacts(
            parsed={"thoughts": "x", "tool_calls": []},
            phrases=["first", "second"],
            history_user_content="hu",
            compact_assistant_text="ca",
            raw_assistant_text="ra",
        )

        persist_and_publish_response(
            artifacts=artifacts,
            user_content="uc",
            append_turn_fn=lambda **kwargs: appended.append(kwargs),
            cancel_requested_event=threading.Event(),
            logger=_LoggerStub(),
            publish_tts_phrase_fn=lambda phrase: published.append(phrase),
        )

        self.assertEqual(len(appended), 1)
        self.assertEqual(published, ["first", "second"])

    def test_persist_and_publish_response_honors_cancel_mid_publish(self):
        appended = []
        published = []
        cancel_requested = threading.Event()

        def _publish(phrase):
            published.append(phrase)
            cancel_requested.set()

        artifacts = ProcessedResponseArtifacts(
            parsed={"thoughts": "x", "tool_calls": []},
            phrases=["first", "second", "third"],
            history_user_content="hu",
            compact_assistant_text="ca",
            raw_assistant_text="ra",
        )

        persist_and_publish_response(
            artifacts=artifacts,
            user_content="uc",
            append_turn_fn=lambda **kwargs: appended.append(kwargs),
            cancel_requested_event=cancel_requested,
            logger=_LoggerStub(),
            publish_tts_phrase_fn=_publish,
        )

        self.assertEqual(len(appended), 1)
        self.assertEqual(published, ["first"])


if __name__ == "__main__":
    unittest.main()
