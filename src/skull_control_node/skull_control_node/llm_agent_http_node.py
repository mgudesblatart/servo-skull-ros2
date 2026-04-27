#!/usr/bin/env python3
"""
llm_agent_http_node.py

ROS 2 node that drives the LLM agent via the axllm HTTP server
(OpenAI-compatible /v1/chat/completions API) instead of a subprocess.

Clean separation from the subprocess path: no prompt-marker scraping,
no ANSI cleaning, no stdin/stdout plumbing.  The model still produces
the same JSON contract so response_parser + say_phrase extraction work
unchanged.

Topic interface expected by the BT node:

  Subscribed:
    /skull_control/llm_input    <- typed LlmInput envelopes from BT
    /llm_agent/control          <- CANCEL / HALT / STOP / RESET commands from BT

  Published:
    /text_to_speech/text_input  -> spoken phrases
    /llm_agent/status           -> typed health / failure events for FSM recovery

Threading model:
  MultiThreadedExecutor (2 threads) + ReentrantCallbackGroup so control_callback
  can interrupt a blocked HTTP call.  The HTTP call itself is not cancellable
  mid-flight (urllib limitation) but the Response is discarded on cancel.
"""

import json
import os
import re
import threading
import time

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from servo_skull_msgs.msg import LlmInput, LlmStatus
from std_msgs.msg import String

from skull_control_node.axllm_http_client import AxllmHttpClient
from skull_control_node.llm_conversation_buffer import ConversationBuffer
from skull_control_node.response_parser import extract_say_phrase_calls, parse_response


# ---------------------------------------------------------------------------
# Compact schema reminder appended to each user turn to reinforce JSON output
# without repeatedly injecting the full policy text into conversation history.
# ---------------------------------------------------------------------------
_JSON_CONTRACT_HINT = (
    'Respond as JSON only with keys: "thoughts" (string) and "tool_calls" (array). '
    'Use say_phrase when speaking is needed.'
)

_SUMMARY_REFINEMENT_HINT = (
    'Return JSON only with keys: "user_preferences", "active_topics", "open_loops", '
    '"assistant_commitments", and "known_facts". Each value must be an array of short strings. '
    'Do not invent facts. Keep only the most important durable context.'
)

_NO_THINK_EXCERPT = (
    "Do not include <think> blocks. Return only the final JSON object.\n/no_think"
)


class LLMAgentHttpNode(Node):
    """HTTP-backed LLM agent node with rolling conversation history."""

    _ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")
    _WHITESPACE_PATTERN = re.compile(r"\s+")
    _BLOCKED_SPEECH_MARKERS = (
        '"thoughts"',
        '"tool_calls"',
        "prompt >>",
    )

    # Valid envelope field values
    _VALID_CHANNELS = {"human", "system", "mixed"}
    _VALID_URGENCIES = {"low", "medium", "high"}
    _SUMMARY_FIELDS = (
        "user_preferences",
        "active_topics",
        "open_loops",
        "assistant_commitments",
        "known_facts",
    )

    def __init__(self) -> None:
        super().__init__("llm_agent_http_node")

        self.callback_group = ReentrantCallbackGroup()
        self.cancel_requested = threading.Event()
        self.request_in_flight = threading.Event()

        self._declare_parameters()
        config = self._load_config()

        self.system_prompt = (
            self._param_str("system_prompt")
            or str(config.get("system_prompt", ""))
        ).strip()

        self._max_history_turns = int(
            self.get_parameter("max_history_turns").value
            or config.get("max_history_turns", 8)
        )
        self._max_window_tokens = int(
            self.get_parameter("max_window_tokens").value
            or config.get("max_window_tokens", 1600)
        )
        self._disable_thinking = bool(
            self.get_parameter("disable_thinking").value
        )
        self._summary_refinement_disable_thinking = bool(
            self.get_parameter("summary_refinement_disable_thinking").value
        )
        self._summary_refinement_max_output_tokens = int(
            self.get_parameter("summary_refinement_max_output_tokens").value
            or config.get("summary_refinement_max_output_tokens", 384)
        )
        self._enable_llm_summary_refinement = bool(
            self.get_parameter("enable_llm_summary_refinement").value
            if self.has_parameter("enable_llm_summary_refinement")
            else config.get("enable_llm_summary_refinement", True)
        )
        self._history_lock = threading.Lock()
        self._conversation_buffer = ConversationBuffer(
            system_prompt=self.system_prompt,
            max_turns=self._max_history_turns,
            max_window_tokens=self._max_window_tokens,
        )
        self._budget_warning_level = "ok"
        if self.system_prompt:
            self.get_logger().info("System prompt loaded into conversation history as first message.")
        else:
            self.get_logger().info("No system prompt configured; conversation starts without a system message.")

        self._server_ready = False
        self.http_client = self._build_http_client(config)
        self._setup_subscriptions()
        self._setup_publishers()

        self.get_logger().info(
            f"LLM Agent HTTP Node starting (will poll for server readiness). "
            f"base_url={self.http_client.base_url}, "
            f"max_history_turns={self._max_history_turns}, "
            f"max_window_tokens={self._max_window_tokens}, "
            f"disable_thinking={self._disable_thinking}, "
            f"summary_refinement_disable_thinking={self._summary_refinement_disable_thinking}, "
            f"summary_refinement_max_output_tokens={self._summary_refinement_max_output_tokens}, "
            f"enable_llm_summary_refinement={self._enable_llm_summary_refinement}."
        )
        self._readiness_timer = self.create_timer(
            0.5, self._poll_server_readiness, callback_group=self.callback_group
        )

    # -----------------------------------------------------------------------
    # Initialisation
    # -----------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("config_path", "")
        self.declare_parameter("axllm_base_url", "http://127.0.0.1:8081")
        self.declare_parameter("axllm_model", "AXERA-TECH/Qwen3-1.7B")
        self.declare_parameter("startup_timeout_sec", 30.0)
        self.declare_parameter("request_timeout_sec", 90.0)
        self.declare_parameter("max_output_tokens", 128)
        self.declare_parameter("max_history_turns", 8)
        self.declare_parameter("max_window_tokens", 1600)
        self.declare_parameter("disable_thinking", True)
        self.declare_parameter("summary_refinement_disable_thinking", False)
        self.declare_parameter("summary_refinement_max_output_tokens", 384)
        self.declare_parameter("enable_llm_summary_refinement", True)
        self.declare_parameter("system_prompt", "")

    def _load_config(self) -> dict:
        config_path = self._param_str("config_path")
        if not config_path:
            return {}
        with open(os.path.expanduser(config_path), "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _param_str(self, name: str) -> str:
        value = self.get_parameter(name).value
        return value.strip() if isinstance(value, str) else ""

    def _build_http_client(self, config: dict) -> AxllmHttpClient:
        base_url = (
            self._param_str("axllm_base_url")
            or str(config.get("axllm_base_url", "http://127.0.0.1:8081"))
        )
        model = (
            self._param_str("axllm_model")
            or str(config.get("axllm_model", "default"))
        )
        startup_timeout = float(
            self.get_parameter("startup_timeout_sec").value
            or config.get("startup_timeout_sec", 30.0)
        )
        request_timeout = float(
            self.get_parameter("request_timeout_sec").value
            or config.get("request_timeout_sec", 90.0)
        )
        max_output_tokens = int(
            self.get_parameter("max_output_tokens").value
            or config.get("max_output_tokens", 128)
        )
        self.get_logger().info(
            f"HTTP backend: base_url={base_url}, model={model}, "
            f"startup_timeout={startup_timeout}s, request_timeout={request_timeout}s, "
            f"max_output_tokens={max_output_tokens}"
        )
        return AxllmHttpClient(
            base_url=base_url,
            model=model,
            startup_timeout_sec=startup_timeout,
            request_timeout_sec=request_timeout,
            max_output_tokens=max_output_tokens,
        )

    def _setup_subscriptions(self) -> None:
        self.prompt_sub = self.create_subscription(
            LlmInput,
            "/skull_control/llm_input",
            self.prompt_callback,
            10,
            callback_group=self.callback_group,
        )
        self.control_sub = self.create_subscription(
            String,
            "/llm_agent/control",
            self.control_callback,
            10,
            callback_group=self.callback_group,
        )

    def _setup_publishers(self) -> None:
        self.tts_pub = self.create_publisher(String, "/text_to_speech/text_input", 10)
        self.status_pub = self.create_publisher(LlmStatus, "/llm_agent/status", 10)

    def _poll_server_readiness(self) -> None:
        """Timer callback: poll /v1/models until 200 OK, then cancel itself."""
        try:
            status, payload = self.http_client._request_json("GET", "/v1/models", timeout_sec=2.0)
            if status == 200 and isinstance(payload, dict):
                self._server_ready = True
                self._readiness_timer.cancel()
                self.get_logger().info("axllm HTTP server is ready — accepting prompts.")
                self._publish_status("ready")
        except Exception as err:
            self.get_logger().debug(f"Server not ready yet: {err}")

    # -----------------------------------------------------------------------
    # Status publishing
    # -----------------------------------------------------------------------

    def _publish_status(self, status: str, *, reason: str = "", detail: str = "") -> None:
        msg = LlmStatus()
        msg.status = status
        msg.reason = reason
        msg.detail = detail
        msg.ts = float(time.time())
        self.status_pub.publish(msg)

    # -----------------------------------------------------------------------
    # History management
    # -----------------------------------------------------------------------

    def _build_messages(self, user_content: str) -> list[dict]:
        """Assemble full OpenAI messages list: history + new user turn."""
        with self._history_lock:
            messages: list[dict] = self._conversation_buffer.get_history_for_prompt()
        messages.append({"role": "user", "content": user_content})
        return messages

    def _append_turn(self, user_content: str, assistant_content: str) -> None:
        """Add a completed turn to the summarized rolling history buffer."""
        with self._history_lock:
            self._conversation_buffer.add_turn(user_content, assistant_content)

        remaining_budget = self._conversation_buffer.get_remaining_budget()
        self._publish_budget_status(remaining_budget)

    def _clear_history(self, reason: str = "") -> None:
        with self._history_lock:
            self._conversation_buffer.clear()
        self._budget_warning_level = "ok"
        self.get_logger().info(f"Conversation history cleared. reason={reason or 'unknown'}")

    def _publish_budget_status(self, remaining_budget: int) -> None:
        low_threshold = max(200, self._max_window_tokens // 4)
        next_level = "ok"
        if remaining_budget < self._conversation_buffer.reset_threshold_tokens:
            next_level = "critical"
        elif remaining_budget < low_threshold:
            next_level = "low"

        if next_level == self._budget_warning_level:
            return

        self._budget_warning_level = next_level
        if next_level == "low":
            detail = f"remaining_tokens={remaining_budget};threshold={low_threshold}"
            self._publish_status("warning", reason="context_budget_low", detail=detail)
            self.get_logger().info(
                f"Conversation buffer budget getting tight: remaining_tokens={remaining_budget}"
            )
        elif next_level == "critical":
            detail = (
                f"remaining_tokens={remaining_budget};"
                f"threshold={self._conversation_buffer.reset_threshold_tokens}"
            )
            self._publish_status("warning", reason="context_budget_critical", detail=detail)
            self.get_logger().warning(
                f"Conversation buffer critically low before reset: remaining_tokens={remaining_budget}"
            )

    def _perform_context_reset(self, pending_user_content: str) -> None:
        with self._history_lock:
            summary_state = self._conversation_buffer.snapshot_summary_state()
            prior_summary_state = self._conversation_buffer.summary_state
            recent_turns = self._conversation_buffer.recent_turns
            summary_text = self._conversation_buffer.generate_reset_summary()
            remaining_before = self._conversation_buffer.get_remaining_budget(pending_user_content)
            refined_summary_state = self._refine_summary_state_with_llm(
                prior_summary_state,
                recent_turns,
                summary_state,
            )
            final_summary_state = refined_summary_state or summary_state
            self._conversation_buffer.reset_with_summary(final_summary_state)
            remaining_after = self._conversation_buffer.get_remaining_budget(pending_user_content)

        self._publish_status(
            "warning",
            reason="context_reset_requested",
            detail=f"remaining_before={remaining_before}",
        )
        backend_reset_ok = self.http_client.reset()
        self._budget_warning_level = "ok"
        self._publish_status(
            "reset",
            reason="context_budget_reset",
            detail=(
                f"remaining_before={remaining_before};"
                f"remaining_after={remaining_after};"
                f"summary_seeded={'yes' if bool(summary_text) else 'no'};"
                f"summary_refined={'yes' if refined_summary_state is not None else 'no'};"
                f"backend_reset_ok={backend_reset_ok}"
            ),
        )
        self.get_logger().warning(
            "Context budget exhausted; reset conversation history with summary seed. "
            f"remaining_before={remaining_before} remaining_after={remaining_after} "
            f"backend_reset_ok={backend_reset_ok}"
        )
        self._publish_budget_status(remaining_after)

    def _build_summary_refinement_messages(
        self,
        prior_summary_state: dict,
        recent_turns: list[tuple[str, str]],
        heuristic_summary_state: dict,
    ) -> list[dict]:
        turn_payload = [
            {
                "user": user_content,
                "assistant": assistant_content,
            }
            for user_content, assistant_content in recent_turns
        ]
        prompt = {
            "existing_summary": prior_summary_state,
            "recent_turns": turn_payload,
            "heuristic_summary": heuristic_summary_state,
        }
        user_content = json.dumps(prompt, ensure_ascii=False, separators=(",", ":"))
        user_content = self._apply_thinking_mode(
            user_content,
            disable_thinking=self._summary_refinement_disable_thinking,
        )
        return [
            {
                "role": "system",
                "content": (
                    "You compress conversation state for a robot assistant. "
                    + _SUMMARY_REFINEMENT_HINT
                ),
            },
            {
                "role": "user",
                "content": user_content,
            },
        ]

    @staticmethod
    def _extract_first_json_object(raw_response: str) -> dict | None:
        """Extract first top-level JSON object from model text, tolerating think noise."""
        sanitized = re.sub(r"<think>.*?</think>", "", raw_response or "", flags=re.DOTALL | re.IGNORECASE)
        sanitized = re.sub(r"</?think>", "", sanitized, flags=re.IGNORECASE).strip()
        if not sanitized:
            return None

        decoder = json.JSONDecoder()
        for index, char in enumerate(sanitized):
            if char != "{":
                continue
            try:
                candidate, _ = decoder.raw_decode(sanitized[index:])
            except json.JSONDecodeError:
                continue
            if isinstance(candidate, dict):
                return candidate
        return None

    def _parse_summary_refinement_response(self, raw_response: str) -> dict | None:
        parsed = self._extract_first_json_object(raw_response)
        if parsed is None:
            return None

        normalized = self._conversation_buffer._normalize_summary_state(parsed)
        if not any(normalized[field] for field in self._SUMMARY_FIELDS):
            return None
        return normalized

    def _refine_summary_state_with_llm(
        self,
        prior_summary_state: dict,
        recent_turns: list[tuple[str, str]],
        heuristic_summary_state: dict,
    ) -> dict | None:
        if not self._enable_llm_summary_refinement:
            return None
        if not any(heuristic_summary_state.get(field) for field in self._SUMMARY_FIELDS):
            return None

        try:
            messages = self._build_summary_refinement_messages(
                prior_summary_state,
                recent_turns,
                heuristic_summary_state,
            )
            raw_response = self.http_client.generate_chat(
                messages,
                max_output_tokens=self._summary_refinement_max_output_tokens,
            )
        except Exception as error:
            self._publish_status(
                "warning",
                reason="context_summary_refine_failed",
                detail=str(error),
            )
            self.get_logger().warning(f"LLM summary refinement failed before reset: {error}")
            return None

        refined = self._parse_summary_refinement_response(raw_response)
        if refined is None:
            self._publish_status(
                "warning",
                reason="context_summary_refine_invalid",
                detail=self._compact_detail(raw_response),
            )
            self.get_logger().warning("LLM summary refinement returned invalid JSON; using heuristic summary.")
            return None

        return refined

    @staticmethod
    def _compact_detail(text: str, max_chars: int = 180) -> str:
        cleaned = re.sub(r"\s+", " ", (text or "").strip())
        if len(cleaned) <= max_chars:
            return cleaned
        return cleaned[: max_chars - 3].rstrip() + "..."

    # -----------------------------------------------------------------------
    # Envelope helpers
    # -----------------------------------------------------------------------

    def _parse_input_envelope(self, msg: LlmInput) -> dict | None:
        channel = str(msg.channel or "").strip().lower()
        source = str(msg.source or "").strip()
        msg_type = str(msg.type or "").strip().lower()
        urgency = str(msg.urgency or "").strip().lower()
        event = str(msg.event or "").strip().lower()
        ts = msg.ts
        # Be tolerant of minor upstream envelope formatting issues.
        if channel not in self._VALID_CHANNELS:
            if event == "human_transcript" and msg_type == "transcript" and source.lower() == "stt":
                channel = "human"
            else:
                return None
        if not source:
            return None
        if not msg_type:
            return None
        if urgency not in self._VALID_URGENCIES:
            return None
        if not isinstance(ts, (int, float)):
            return None

        text = msg.text
        payload = {}
        payload_json = msg.payload_json
        if isinstance(payload_json, str) and payload_json.strip():
            try:
                parsed_payload = json.loads(payload_json)
                if isinstance(parsed_payload, dict):
                    payload = parsed_payload
            except json.JSONDecodeError:
                payload = {}
        return {
            "channel": channel,
            "source": source,
            "type": msg_type,
            "event": event,
            "urgency": urgency,
            "ts": float(ts),
            "text": text.strip() if isinstance(text, str) else "",
            "payload": payload,
        }

    @staticmethod
    def _build_user_content_raw(envelope: dict) -> str:
        """Format envelope into a compact user-turn payload."""
        if envelope["text"]:
            if envelope["channel"] == "human":
                return (
                    "Input Class: DIRECT_HUMAN\n"
                    f"User transcript: {envelope['text']}\n\n"
                    f"{_JSON_CONTRACT_HINT}"
                )

        # System event fallback lane (currently disabled upstream, kept for future use).
        system_request = envelope["text"] or (envelope.get("event") or "event_update")
        return (
            "Input Class: SYSTEM_EVENT\n"
            f"System Request: {system_request}\n\n"
            f"{_JSON_CONTRACT_HINT}"
        )

    def _apply_thinking_mode(self, content: str, disable_thinking: bool | None = None) -> str:
        text = (content or "").strip()
        should_disable = self._disable_thinking if disable_thinking is None else bool(disable_thinking)
        if not should_disable:
            return text
        if _NO_THINK_EXCERPT in text:
            return text
        return f"{text}\n\n{_NO_THINK_EXCERPT}".strip()

    @staticmethod
    def _normalize_assistant_history_contract(parsed: dict, spoken_phrases: list[str]) -> str:
        """
        Persist assistant turns in strict contract shape so history stays
        consistent with the model's required output format.
        """
        thoughts = parsed.get("thoughts")
        thoughts_text = thoughts.strip() if isinstance(thoughts, str) else ""

        tool_calls: list[dict] = []
        if spoken_phrases:
            tool_calls = [{"say_phrase": {"msg": spoken_phrases[0]}}]
        elif isinstance(parsed.get("tool_calls"), list):
            tool_calls = parsed["tool_calls"]

        contract_obj = {
            "thoughts": thoughts_text,
            "tool_calls": tool_calls,
        }
        return json.dumps(contract_obj, separators=(",", ":"), ensure_ascii=False)

    # -----------------------------------------------------------------------
    # Output sanitization
    # -----------------------------------------------------------------------

    def _sanitize_spoken_phrase(self, phrase: str) -> str:
        """Reject JSON/log contamination; clean whitespace."""
        cleaned = self._ANSI_ESCAPE_PATTERN.sub("", phrase or "")
        cleaned = cleaned.strip().strip('"').strip("'")
        cleaned = self._WHITESPACE_PATTERN.sub(" ", cleaned).strip()
        if not cleaned:
            return ""
        lowered = cleaned.lower()
        if any(marker in lowered for marker in self._BLOCKED_SPEECH_MARKERS):
            return ""
        if "{" in cleaned or "}" in cleaned:
            return ""
        return cleaned

    # -----------------------------------------------------------------------
    # Control callback
    # -----------------------------------------------------------------------

    def control_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command not in {"CANCEL", "HALT", "STOP", "RESET"}:
            return

        if command == "RESET":
            self.cancel_requested.set()
            self._clear_history(reason="voice_reset")
            backend_reset_ok = self.http_client.reset()
            self._publish_status(
                "reset",
                reason="control_reset",
                detail="backend_reset_ok" if backend_reset_ok else "history_reset_only",
            )
            self.get_logger().warning(
                f"LLM HTTP control: RESET — history cleared (backend reset={backend_reset_ok})."
            )
            if not self.request_in_flight.is_set():
                self.cancel_requested.clear()
            return

        self.cancel_requested.set()
        self.get_logger().warning(f"LLM HTTP control: {command} — marking cancel.")
        if not self.request_in_flight.is_set():
            self.cancel_requested.clear()

    # -----------------------------------------------------------------------
    # Main prompt callback
    # -----------------------------------------------------------------------

    def prompt_callback(self, msg: LlmInput) -> None:
        if not msg.channel.strip() and not msg.text.strip():
            return
        if not self._server_ready:
            self.get_logger().warning("Prompt received but axllm server not ready yet; dropping.")
            return

        envelope = self._parse_input_envelope(msg)
        if envelope is None:
            self.get_logger().warning("Dropping invalid LlmInput envelope.")
            return

        channel = envelope["channel"]
        self.get_logger().info(
            f"llm_input envelope — channel={channel} "
            f"type={envelope['type']} event={envelope['event'] or 'none'} "
            f"source={envelope['source']}"
        )
        # Minimal mode contract: only handle direct STT human transcripts.
        if not (
            channel == "human"
            and envelope.get("source") == "stt"
            and envelope.get("type") == "transcript"
            and envelope.get("event") == "human_transcript"
        ):
            self.get_logger().debug(
                f"Dropping non-human/non-STT envelope in minimal mode (channel={channel}, event={envelope['event']})."
            )
            return

        user_content = self._build_user_content_raw(envelope)
        user_content = self._apply_thinking_mode(user_content)
        remaining_budget = self._conversation_buffer.get_remaining_budget(user_content)
        self._publish_budget_status(remaining_budget)
        if self._conversation_buffer.should_reset(user_content):
            self._perform_context_reset(user_content)

        try:
            self.request_in_flight.set()
            messages = self._build_messages(user_content)
            raw_response = self.http_client.generate_chat(messages)
        except Exception as error:
            if self.cancel_requested.is_set():
                self.get_logger().warning(f"HTTP request cancelled: {error}")
                self._publish_status("cancelled", reason="control_cancel", detail=str(error))
            else:
                self.get_logger().error(f"HTTP inference error: {error}")
                self._publish_status(
                    "error", reason="http_inference_error", detail=str(error)
                )
            return
        finally:
            self.request_in_flight.clear()
            if self.cancel_requested.is_set():
                self.cancel_requested.clear()
                self.get_logger().warning("HTTP response received but cancel was set; dropping.")
                return

        self.get_logger().info(f"Model response: {raw_response}")

        parsed = parse_response(raw_response)
        if parsed is None:
            self.get_logger().error("Failed to parse model response into JSON contract.")
            self._publish_status(
                "parse_error",
                reason="invalid_model_output",
                detail="Response failed JSON-contract parsing.",
            )
            return

        phrases = [
            p for p in (self._sanitize_spoken_phrase(p) for p in extract_say_phrase_calls(parsed))
            if p
        ]

        # Preserve the raw assistant payload in history so subsequent requests
        # remain append-compatible for server-side KV cache reuse.
        assistant_text = raw_response
        if not isinstance(assistant_text, str) or not assistant_text.strip():
            assistant_text = self._normalize_assistant_history_contract(parsed, phrases)
        if assistant_text.strip():
            self._append_turn(user_content, assistant_text)

        for phrase in phrases:
            if self.cancel_requested.is_set():
                self.get_logger().warning("Cancel received during publish loop; stopping.")
                break
            tts_msg = String()
            tts_msg.data = phrase
            self.tts_pub.publish(tts_msg)
            self.get_logger().info(f"Published TTS phrase: {phrase}")

    def destroy_node(self) -> bool:
        self.http_client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LLMAgentHttpNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
