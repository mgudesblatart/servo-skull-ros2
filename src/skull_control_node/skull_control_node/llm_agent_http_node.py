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
    /skull_control/llm_input    <- structured JSON envelopes from BT
    /llm_agent/control          <- CANCEL / HALT / STOP commands from BT

  Published:
    /text_to_speech/text_input  -> spoken phrases
    /llm_agent/status           -> health / failure events for FSM recovery

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
from std_msgs.msg import String

from skull_control_node.axllm_http_client import AxllmHttpClient
from skull_control_node.response_parser import extract_say_phrase_calls, parse_response


# ---------------------------------------------------------------------------
# JSON output contract appended to every user message so the model knows
# what shape of response to produce.
# ---------------------------------------------------------------------------
_JSON_CONTRACT = (
    "Return exactly one JSON object and nothing else. "
    "Required keys: thoughts (string), tool_calls (array). "
    'If speaking is appropriate, include tool_calls like '
    '[{"say_phrase": {"msg": "..."}}].'
)


class LLMAgentHttpNode(Node):
    """HTTP-backed LLM agent node with rolling conversation history."""

    _ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")
    _WHITESPACE_PATTERN = re.compile(r"\s+")
    _BLOCKED_SPEECH_MARKERS = (
        '"thoughts"',
        '"tool_calls"',
        '"final_output"',
        "prompt >>",
    )

    # Valid envelope field values
    _VALID_CHANNELS = {"human", "system", "mixed"}
    _VALID_URGENCIES = {"low", "medium", "high"}

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

        # Rolling history: list of {"role": "system"|"user"|"assistant", "content": str}
        self._history: list[dict] = []
        if self.system_prompt:
            self._history.append({"role": "system", "content": self.system_prompt})
            self.get_logger().info("System prompt loaded into conversation history as first message.")
        else:
            self.get_logger().info("No system prompt configured; conversation starts without a system message.")
        self._history_lock = threading.Lock()
        self._max_history_turns = int(
            self.get_parameter("max_history_turns").value
            or config.get("max_history_turns", 8)
        )

        self._server_ready = False
        self.http_client = self._build_http_client(config)
        self._setup_subscriptions()
        self._setup_publishers()

        self.get_logger().info(
            f"LLM Agent HTTP Node starting (will poll for server readiness). "
            f"base_url={self.http_client.base_url}, "
            f"max_history_turns={self._max_history_turns}."
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
        self.declare_parameter("request_timeout_sec", 60.0)
        self.declare_parameter("max_history_turns", 8)
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
            or config.get("request_timeout_sec", 60.0)
        )
        self.get_logger().info(
            f"HTTP backend: base_url={base_url}, model={model}, "
            f"startup_timeout={startup_timeout}s, request_timeout={request_timeout}s"
        )
        return AxllmHttpClient(
            base_url=base_url,
            model=model,
            startup_timeout_sec=startup_timeout,
            request_timeout_sec=request_timeout,
        )

    def _setup_subscriptions(self) -> None:
        self.prompt_sub = self.create_subscription(
            String,
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
        self.status_pub = self.create_publisher(String, "/llm_agent/status", 10)

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
        msg = String()
        msg.data = json.dumps(
            {"status": status, "reason": reason, "detail": detail, "ts": time.time()},
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    # -----------------------------------------------------------------------
    # History management
    # -----------------------------------------------------------------------

    def _build_messages(self, user_content: str) -> list[dict]:
        """Assemble full OpenAI messages list: history + new user turn."""
        with self._history_lock:
            messages: list[dict] = list(self._history)
        messages.append({"role": "user", "content": user_content})
        return messages

    def _append_turn(self, user_content: str, assistant_content: str) -> None:
        """Add a completed turn to history, trimming oldest pairs to stay within window."""
        with self._history_lock:
            self._history.append({"role": "user", "content": user_content})
            self._history.append({"role": "assistant", "content": assistant_content})
            # Keep the leading system message if present; trim only conversation turns.
            if self._history and self._history[0].get("role") == "system":
                system_msg = self._history[0]
                turns = self._history[1:]
                max_turn_msgs = self._max_history_turns * 2
                if len(turns) > max_turn_msgs:
                    turns = turns[-max_turn_msgs:]
                self._history = [system_msg] + turns
            else:
                max_msgs = self._max_history_turns * 2
                if len(self._history) > max_msgs:
                    self._history = self._history[-max_msgs:]

    def _clear_history(self, reason: str = "") -> None:
        with self._history_lock:
            self._history.clear()
            if self.system_prompt:
                self._history.append({"role": "system", "content": self.system_prompt})
        self.get_logger().info(f"Conversation history cleared. reason={reason or 'unknown'}")

    # -----------------------------------------------------------------------
    # Envelope helpers
    # -----------------------------------------------------------------------

    def _parse_input_envelope(self, raw: str) -> dict | None:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return None
        if not isinstance(data, dict):
            return None
        channel = data.get("channel")
        source = data.get("source")
        msg_type = data.get("type")
        urgency = data.get("urgency")
        ts = data.get("ts")
        if channel not in self._VALID_CHANNELS:
            return None
        if not isinstance(source, str) or not source.strip():
            return None
        if not isinstance(msg_type, str) or not msg_type.strip():
            return None
        if urgency not in self._VALID_URGENCIES:
            return None
        if not isinstance(ts, (int, float)):
            return None

        text = data.get("text")
        payload = data.get("payload")
        return {
            "channel": channel,
            "source": source.strip(),
            "type": msg_type.strip(),
            "event": str(data.get("event", "") or "").strip(),
            "urgency": urgency,
            "ts": float(ts),
            "text": text.strip() if isinstance(text, str) else "",
            "payload": payload if isinstance(payload, dict) else {},
        }

    @staticmethod
    def _build_user_content(envelope: dict) -> str:
        """Format the envelope into a natural user-turn content string."""
        parts = []
        if envelope["text"]:
            # Human transcript: just the words, plus the JSON contract
            if envelope["channel"] == "human":
                return f"{envelope['text']} {_JSON_CONTRACT}"
            parts.append(f"text={envelope['text']}")

        # Non-human event: describe the context
        meta = (
            f"channel={envelope['channel']} source={envelope['source']} "
            f"event={envelope['event'] or 'none'} urgency={envelope['urgency']}"
        )
        if envelope["payload"]:
            meta += f" payload={json.dumps(envelope['payload'], separators=(',', ':'))}"
        parts.insert(0, f"System context: {meta}")
        return " ".join(parts) + " " + _JSON_CONTRACT

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
        if command not in {"CANCEL", "HALT", "STOP"}:
            return
        self.cancel_requested.set()
        self.get_logger().warning(f"LLM HTTP control: {command} — marking cancel.")
        if not self.request_in_flight.is_set():
            self.cancel_requested.clear()

    # -----------------------------------------------------------------------
    # Main prompt callback
    # -----------------------------------------------------------------------

    def prompt_callback(self, msg: String) -> None:
        raw_input = msg.data.strip()
        if not raw_input:
            return
        if not self._server_ready:
            self.get_logger().warning("Prompt received but axllm server not ready yet; dropping.")
            return

        envelope = self._parse_input_envelope(raw_input)

        if envelope is None:
            # Plain-text fallback (shouldn't normally happen)
            self.get_logger().info(f"llm_input plain-text fallback: {raw_input[:80]}")
            user_content = f"{raw_input} {_JSON_CONTRACT}"
            channel = "human"
        else:
            channel = envelope["channel"]
            self.get_logger().info(
                f"llm_input envelope — channel={channel} "
                f"type={envelope['type']} event={envelope['event'] or 'none'} "
                f"source={envelope['source']}"
            )
            # Only call the LLM for human input; silently ack system events
            if channel != "human":
                allow_tts = (
                    isinstance(envelope.get("payload"), dict)
                    and envelope["payload"].get("allow_tts") is True
                )
                if not allow_tts:
                    self.get_logger().debug(
                        f"Silently acking non-human envelope (event={envelope['event']})."
                    )
                    return
            user_content = self._build_user_content(envelope)

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

        self.get_logger().info(f"Model response: {raw_response[:200]}")

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

        # Human input must always produce at least one spoken reply
        if channel == "human" and not phrases:
            final_output = parsed.get("final_output")
            fallback = self._sanitize_spoken_phrase(
                final_output if isinstance(final_output, str) else ""
            )
            if not fallback:
                fallback = "I got that."
            phrases = [fallback]
            self.get_logger().warning("No say_phrase in response; using final_output fallback.")

        # Store the turn in history (use assistant's actual spoken content)
        assistant_text = " ".join(phrases) if phrases else (parsed.get("final_output") or "")
        if isinstance(assistant_text, str) and assistant_text.strip():
            self._append_turn(user_content, assistant_text.strip())

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
