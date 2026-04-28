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
from skull_control_node.utils.llm_conversation_buffer import ConversationBuffer
from skull_control_node.utils.context_input_helpers import (
    build_user_content_raw,
    build_history_user_content,
    parse_input_envelope,
)
from skull_control_node.utils.context_reset_manager import ContextResetManager
from skull_control_node.utils.formatting_parsing_helpers import (
    compact_detail,
    normalize_assistant_history_contract,
    sanitize_spoken_phrase,
)
from skull_control_node.utils.generation_stage import run_generation
from skull_control_node.utils.prompt_intake_stage import prepare_prompt_intake
from skull_control_node.utils.response_output_stage import persist_and_publish_response
from skull_control_node.utils.response_processing_stage import process_model_response
from skull_control_node.utils.response_parser import (
    extract_say_phrase_calls,
    parse_response,
)
# ---------------------------------------------------------------------------
# Compact schema reminder appended to each user turn to reinforce JSON output
# without repeatedly injecting the full policy text into conversation history.
# ---------------------------------------------------------------------------
_JSON_CONTRACT_HINT = (
    'Respond as JSON only with keys: "thoughts" (string) and "tool_calls" (array). '
    "Use say_phrase when speaking is needed."
)

_SUMMARY_REFINEMENT_HINT = (
    'Return JSON only with keys: "user_preferences", "active_topics", "open_loops", '
    '"assistant_commitments", and "known_facts". Each value must be an array of short strings. '
    "Do not invent facts. Keep only the most important durable context."
)

_NO_THINK_EXCERPT = (
    "Do not include <think> blocks. Return only the final JSON object.\n/no_think"
)


class LLMAgentHttpNode(Node):
    """HTTP-backed LLM agent node with rolling conversation history."""

    _BLOCKED_SPEECH_MARKERS = (
        '"thoughts"',
        '"tool_calls"',
        "prompt >>",
    )

    # Boilerplate patterns to strip from history to prevent feedback loop
    _BOILERPLATE_PATTERNS = (
        r"\bthis unit is here to assist with any queries regarding the imperium",
        r"\bthis unit is at your service",
        r"\bthis unit is tracking inputs and responding to commands",
        r"\bhow may i assist you further",
        r"\bfurther orders, master",
        r"\bthis unit stands ready",
        r"\bawait your next command",
        r"\bmore assistance needed",
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
            self._param_str("system_prompt") or str(config.get("system_prompt", ""))
        ).strip()

        self._max_history_turns = int(
            self.get_parameter("max_history_turns").value
            or config.get("max_history_turns", 8)
        )
        self._max_window_tokens = int(
            self.get_parameter("max_window_tokens").value
            or config.get("max_window_tokens", 1600)
        )
        self._disable_thinking = bool(self.get_parameter("disable_thinking").value)
        self._summary_refinement_disable_thinking = bool(
            self.get_parameter("summary_refinement_disable_thinking").value
        )
        self._summary_refinement_request_timeout_sec = float(
            self.get_parameter("summary_refinement_request_timeout_sec").value
            or config.get("summary_refinement_request_timeout_sec", 180.0)
        )
        self._summary_refinement_max_output_tokens = int(
            self.get_parameter("summary_refinement_max_output_tokens").value
            or config.get("summary_refinement_max_output_tokens", 384)
        )
        self._summary_refinement_unbounded_when_thinking = bool(
            self.get_parameter("summary_refinement_unbounded_when_thinking").value
            if self.has_parameter("summary_refinement_unbounded_when_thinking")
            else config.get("summary_refinement_unbounded_when_thinking", True)
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
        self._server_ready = False
        self.http_client = self._build_http_client(config)
        self._context_manager = ContextResetManager(
            system_prompt=self.system_prompt,
            max_window_tokens=self._max_window_tokens,
            conversation_buffer=self._conversation_buffer,
            http_client=self.http_client,
            history_lock=self._history_lock,
            publish_status=self._publish_status,
            logger=self.get_logger(),
            summary_fields=self._SUMMARY_FIELDS,
            summary_refinement_hint=_SUMMARY_REFINEMENT_HINT,
            no_think_excerpt=_NO_THINK_EXCERPT,
            default_disable_thinking=self._disable_thinking,
            summary_refinement_disable_thinking=self._summary_refinement_disable_thinking,
            summary_refinement_request_timeout_sec=self._summary_refinement_request_timeout_sec,
            summary_refinement_max_output_tokens=self._summary_refinement_max_output_tokens,
            summary_refinement_unbounded_when_thinking=self._summary_refinement_unbounded_when_thinking,
            enable_llm_summary_refinement=self._enable_llm_summary_refinement,
        )
        if self.system_prompt:
            self.get_logger().info(
                "System prompt loaded into conversation history as first message."
            )
        else:
            self.get_logger().info(
                "No system prompt configured; conversation starts without a system message."
            )

        self._setup_subscriptions()
        self._setup_publishers()

        self.get_logger().info(
            f"LLM Agent HTTP Node starting (will poll for server readiness). "
            f"base_url={self.http_client.base_url}, "
            f"max_history_turns={self._max_history_turns}, "
            f"max_window_tokens={self._max_window_tokens}, "
            f"disable_thinking={self._disable_thinking}, "
            f"summary_refinement_disable_thinking={self._summary_refinement_disable_thinking}, "
            f"summary_refinement_request_timeout_sec={self._summary_refinement_request_timeout_sec}, "
            f"summary_refinement_max_output_tokens={self._summary_refinement_max_output_tokens}, "
            f"summary_refinement_unbounded_when_thinking={self._summary_refinement_unbounded_when_thinking}, "
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
        self.declare_parameter("summary_refinement_request_timeout_sec", 180.0)
        self.declare_parameter("max_output_tokens", 128)
        self.declare_parameter("max_history_turns", 8)
        self.declare_parameter("max_window_tokens", 1600)
        self.declare_parameter("disable_thinking", True)
        self.declare_parameter("summary_refinement_disable_thinking", False)
        self.declare_parameter("summary_refinement_max_output_tokens", 384)
        self.declare_parameter("summary_refinement_unbounded_when_thinking", True)
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
        base_url = self._param_str("axllm_base_url") or str(
            config.get("axllm_base_url", "http://127.0.0.1:8081")
        )
        model = self._param_str("axllm_model") or str(
            config.get("axllm_model", "default")
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
            status, payload = self.http_client._request_json(
                "GET", "/v1/models", timeout_sec=2.0
            )
            if status == 200 and isinstance(payload, dict):
                self._server_ready = True
                self._readiness_timer.cancel()
                self.get_logger().info(
                    "axllm HTTP server is ready — accepting prompts."
                )
                self._publish_status("ready")
        except Exception as err:
            self.get_logger().debug(f"Server not ready yet: {err}")

    # -----------------------------------------------------------------------
    # Status publishing
    # -----------------------------------------------------------------------

    def _publish_status(
        self, status: str, *, reason: str = "", detail: str = ""
    ) -> None:
        msg = LlmStatus()
        msg.status = status
        msg.reason = reason
        msg.detail = detail
        msg.ts = float(time.time())
        self.status_pub.publish(msg)

    # -----------------------------------------------------------------------
    # History management
    # -----------------------------------------------------------------------

    def _ensure_context_manager(self) -> ContextResetManager:
        if hasattr(self, "_context_manager"):
            return self._context_manager

        # Legacy/test fallback path for __new__-constructed nodes.
        self._context_manager = ContextResetManager(
            system_prompt=getattr(self, "system_prompt", ""),
            max_window_tokens=getattr(self, "_max_window_tokens", 1600),
            conversation_buffer=self._conversation_buffer,
            http_client=self.http_client,
            history_lock=self._history_lock,
            publish_status=self._publish_status,
            logger=self.get_logger(),
            summary_fields=self._SUMMARY_FIELDS,
            summary_refinement_hint=_SUMMARY_REFINEMENT_HINT,
            no_think_excerpt=_NO_THINK_EXCERPT,
            default_disable_thinking=getattr(self, "_disable_thinking", True),
            summary_refinement_disable_thinking=getattr(
                self, "_summary_refinement_disable_thinking", False
            ),
            summary_refinement_request_timeout_sec=getattr(
                self, "_summary_refinement_request_timeout_sec", 180.0
            ),
            summary_refinement_max_output_tokens=getattr(
                self, "_summary_refinement_max_output_tokens", 384
            ),
            summary_refinement_unbounded_when_thinking=getattr(
                self, "_summary_refinement_unbounded_when_thinking", True
            ),
            enable_llm_summary_refinement=getattr(
                self, "_enable_llm_summary_refinement", True
            ),
        )

        if "_append_history" in self.__dict__:
            self._context_manager.append_history = list(self.__dict__["_append_history"])
        if "_budget_warning_level" in self.__dict__:
            self._context_manager.budget_warning_level = self.__dict__["_budget_warning_level"]
        return self._context_manager

    @property
    def _append_history(self) -> list[dict]:
        return self._ensure_context_manager().append_history

    @_append_history.setter
    def _append_history(self, value: list[dict]) -> None:
        if hasattr(self, "_context_manager"):
            self._context_manager.append_history = list(value)
        else:
            self.__dict__["_append_history"] = list(value)

    @property
    def _budget_warning_level(self) -> str:
        return self._ensure_context_manager().budget_warning_level

    @_budget_warning_level.setter
    def _budget_warning_level(self, value: str) -> None:
        if hasattr(self, "_context_manager"):
            self._context_manager.budget_warning_level = value
        else:
            self.__dict__["_budget_warning_level"] = value

    def _build_messages(self, user_content: str) -> list[dict]:
        """Assemble append-only raw history + new user turn for LLM generation."""
        return self._ensure_context_manager().build_messages(user_content)

    def _append_turn(
        self,
        *,
        prompt_user_content: str,
        compact_user_content: str,
        raw_assistant_content: str,
        compact_assistant_content: str,
    ) -> None:
        """Append raw turn for model reuse and compact turn for reset summarization."""
        self._ensure_context_manager().append_turn(
            prompt_user_content=prompt_user_content,
            compact_user_content=compact_user_content,
            raw_assistant_content=raw_assistant_content,
            compact_assistant_content=compact_assistant_content,
        )

    def _clear_history(self, reason: str = "") -> None:
        self._ensure_context_manager().clear_history(reason)

    def _reset_append_history_root(self, summary_text: str = "") -> None:
        self._ensure_context_manager().reset_append_history_root(summary_text)

    def _get_append_remaining_budget(self, pending_user_content: str = "") -> int:
        return self._ensure_context_manager().get_append_remaining_budget(
            pending_user_content
        )

    def _publish_budget_status(self, remaining_budget: int) -> None:
        self._ensure_context_manager().publish_budget_status(remaining_budget)

    def _perform_context_reset(self, pending_user_content: str) -> None:
        self._ensure_context_manager().perform_context_reset(pending_user_content)

    def _refine_summary_state_with_llm(
        self,
        prior_summary_state: dict,
        untruncated_turns: list[tuple[str, str]] | None = None,
        heuristic_summary_state: dict | None = None,
        recent_turns: list[tuple[str, str]] | None = None,
    ) -> dict | None:
        turns = untruncated_turns if untruncated_turns is not None else (recent_turns or [])
        summary_state = heuristic_summary_state or {}
        return self._ensure_context_manager().refine_summary_state_with_llm(
            prior_summary_state,
            turns,
            summary_state,
        )

    def _parse_input_envelope(self, msg: LlmInput) -> dict | None:
        """Compatibility wrapper retained for tests and legacy call sites."""
        return parse_input_envelope(
            msg,
            valid_channels=self._VALID_CHANNELS,
            valid_urgencies=self._VALID_URGENCIES,
        )

    @staticmethod
    def _build_user_content_raw(envelope: dict) -> str:
        """Compatibility wrapper retained for tests and legacy call sites."""
        return build_user_content_raw(envelope, _JSON_CONTRACT_HINT)

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
        intake = prepare_prompt_intake(
            msg,
            server_ready=self._server_ready,
            valid_channels=self._VALID_CHANNELS,
            valid_urgencies=self._VALID_URGENCIES,
            json_contract_hint=_JSON_CONTRACT_HINT,
            default_disable_thinking=self._disable_thinking,
            no_think_excerpt=_NO_THINK_EXCERPT,
        )
        if not intake.accepted:
            logger = self.get_logger()
            if intake.log_message:
                if intake.log_level == "warning":
                    logger.warning(intake.log_message)
                elif intake.log_level == "info":
                    logger.info(intake.log_message)
                elif intake.log_level == "error":
                    logger.error(intake.log_message)
                else:
                    logger.debug(intake.log_message)
            return

        envelope = intake.envelope
        user_content = intake.user_content
        channel = envelope["channel"]
        self.get_logger().info(
            f"llm_input envelope — channel={channel} "
            f"type={envelope['type']} event={envelope['event'] or 'none'} "
            f"source={envelope['source']}"
        )

        remaining_budget = self._get_append_remaining_budget(user_content)
        self._publish_budget_status(remaining_budget)
        if remaining_budget < self._conversation_buffer.reset_threshold_tokens:
            self._perform_context_reset(user_content)

        generation = run_generation(
            user_content=user_content,
            build_messages=self._build_messages,
            http_generate_chat=self.http_client.generate_chat,
            request_in_flight_event=self.request_in_flight,
            cancel_requested_event=self.cancel_requested,
            logger=self.get_logger(),
            publish_status=self._publish_status,
        )
        if not generation.ok:
            return

        raw_response = generation.raw_response
        self.get_logger().info(f"Model response: {raw_response}")

        artifacts = process_model_response(
            raw_response=raw_response,
            envelope=envelope,
            user_content=user_content,
            parse_response_fn=parse_response,
            extract_say_phrase_calls_fn=extract_say_phrase_calls,
            sanitize_spoken_phrase_fn=sanitize_spoken_phrase,
            build_history_user_content_fn=build_history_user_content,
            compact_detail_fn=compact_detail,
            normalize_assistant_history_contract_fn=normalize_assistant_history_contract,
            boilerplate_patterns=self._BOILERPLATE_PATTERNS,
            blocked_speech_markers=self._BLOCKED_SPEECH_MARKERS,
            logger=self.get_logger(),
            publish_status=self._publish_status,
        )
        if artifacts is None:
            return

        persist_and_publish_response(
            artifacts=artifacts,
            user_content=user_content,
            append_turn_fn=self._append_turn,
            cancel_requested_event=self.cancel_requested,
            logger=self.get_logger(),
            publish_tts_phrase_fn=self._publish_tts_phrase,
        )

    def _publish_tts_phrase(self, phrase: str) -> None:
        tts_msg = String()
        tts_msg.data = phrase
        self.tts_pub.publish(tts_msg)

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
