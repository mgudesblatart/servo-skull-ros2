#!/usr/bin/env python3
"""
llm_agent_axcl_node.py

ROS 2 node that wraps the AXCL runtime (llama.cpp or similar) as a
conversational LLM agent. It:

  - Subscribes to /skull_control/llm_input for structured JSON envelopes
    (human speech, system events) gated through skull_control_bt_node.
  - Subscribes to /llm_agent_axcl/control for CANCEL/HALT/STOP commands.
  - Drives the AXCL runtime subprocess via AxclRuntimeClient (stdin/stdout).
  - Parses model output using response_parser to extract say_phrase tool calls.
  - Publishes extracted speech phrases to /text_to_speech/text_input.

Threading model:
  Uses MultiThreadedExecutor (2 threads) + ReentrantCallbackGroup so that
  control commands can preempt an in-flight prompt_callback without deadlock.
  A threading.RLock guards all runtime start/stop operations.
"""

import json
import os
import re
import threading

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from skull_control_node.axcl_runtime_client import AxclRuntimeClient
from skull_control_node.response_parser import extract_say_phrase_calls, parse_response


class LLMAgentAxclNode(Node):
    # Compiled once at class level; used to strip runtime log noise from output
    _RUNTIME_LOG_LINE_PATTERN = re.compile(r"^\[[A-Z]\]\[")
    _ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")

    # Valid values for envelope fields; anything else is dropped
    _VALID_CHANNELS = {"human", "system", "mixed"}
    _VALID_URGENCIES = {"low", "medium", "high"}

    def __init__(self) -> None:
        super().__init__("llm_agent_axcl_node")

        # ReentrantCallbackGroup allows control_callback to fire while
        # prompt_callback is blocking on runtime inference
        self.callback_group = ReentrantCallbackGroup()

        # cancel_requested: set by control_callback, checked by prompt_callback
        # request_in_flight: set when prompt_callback enters inference, cleared on exit
        self.cancel_requested = threading.Event()
        self.request_in_flight = threading.Event()

        # Serialises all runtime start/stop/restart operations
        self.runtime_reset_lock = threading.RLock()

        self._declare_parameters()
        config = self._load_config()
        self.runtime_client = self._build_runtime_client(config)
        self.inline_system_prompt = self._resolve_inline_system_prompt(config)
        self.system_prompt = config.get("system_prompt", "").strip()

        self._setup_subscriptions()
        self._setup_publishers()

        self.get_logger().info("Starting AXCL runtime client...")
        self.runtime_client.start()
        self.get_logger().info("LLM Agent AXCL Node started.")

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("config_path", "")
        self.declare_parameter("runtime_command", "")
        self.declare_parameter("runtime_cwd", "")
        self.declare_parameter("prompt_marker", "prompt >>")
        self.declare_parameter("startup_timeout_sec", 120.0)
        self.declare_parameter("response_timeout_sec", 60.0)
        self.declare_parameter("inline_system_prompt", "")

    def _build_runtime_client(self, config: dict) -> AxclRuntimeClient:
        """Construct AxclRuntimeClient from ROS params (preferred) or config YAML."""
        runtime_command = self._param_str("runtime_command") or config.get("runtime_command", "")
        runtime_cwd = self._param_str("runtime_cwd") or config.get("runtime_cwd", "")
        prompt_marker = self._param_str("prompt_marker") or config.get("prompt_marker", "prompt >>")
        startup_timeout = float(self.get_parameter("startup_timeout_sec").value or config.get("startup_timeout_sec", 120.0))
        response_timeout = float(self.get_parameter("response_timeout_sec").value or config.get("response_timeout_sec", 60.0))

        if not runtime_command:
            raise RuntimeError(
                "runtime_command is not configured. Set it via ROS param or axcl_servo_skull.yaml."
            )

        # Inject system prompt via environment if the runtime reads it from SYSTEM_PROMPT
        env_overrides: dict[str, str] = {}
        system_prompt = config.get("system_prompt", "").strip()
        inline = self._resolve_inline_system_prompt(config)
        if not inline and system_prompt:
            if os.environ.get("SYSTEM_PROMPT", "").strip():
                self.get_logger().info("Using SYSTEM_PROMPT from environment.")
            else:
                env_overrides["SYSTEM_PROMPT"] = system_prompt
                self.get_logger().info("Injecting system_prompt from config YAML via env.")

        return AxclRuntimeClient(
            command=os.path.expanduser(runtime_command),
            cwd=os.path.expanduser(runtime_cwd) if runtime_cwd else None,
            env_overrides=env_overrides,
            prompt_marker=prompt_marker,
            startup_timeout_sec=startup_timeout,
            response_timeout_sec=response_timeout,
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
            "/llm_agent_axcl/control",
            self.control_callback,
            10,
            callback_group=self.callback_group,
        )

    def _setup_publishers(self) -> None:
        self.tts_pub = self.create_publisher(String, "/text_to_speech/text_input", 10)

    # -----------------------------------------------------------------------
    # Runtime lifecycle
    # -----------------------------------------------------------------------

    def _reset_runtime_locked(self, reason: str) -> None:
        """
        Close and restart the AXCL subprocess.
        Caller MUST hold runtime_reset_lock before calling this.
        """
        try:
            self.runtime_client.close()
        except Exception as error:
            self.get_logger().warning(f"Runtime close during {reason} raised: {error}")
        try:
            self.runtime_client.start()
            self.get_logger().info(f"AXCL runtime restarted ({reason}).")
        except Exception as error:
            self.get_logger().error(f"Failed to restart AXCL runtime ({reason}): {error}")

    def destroy_node(self) -> bool:
        with self.runtime_reset_lock:
            self.runtime_client.close()
        return super().destroy_node()

    # -----------------------------------------------------------------------
    # Control callback (runs on its own thread via ReentrantCallbackGroup)
    # -----------------------------------------------------------------------

    def control_callback(self, msg: String) -> None:
        """
        Handle CANCEL / HALT / STOP from /llm_agent_axcl/control.

        Sets cancel_requested so an in-flight prompt_callback will bail out
        before publishing. If a request is actually in flight, restarts the
        runtime to abort the blocked subprocess read.
        """
        command = msg.data.strip().upper()
        if command not in {"CANCEL", "HALT", "STOP"}:
            return

        self.cancel_requested.set()
        self.get_logger().warning(f"LLM control: {command} — cancelling active request.")

        if not self.request_in_flight.is_set():
            self.get_logger().info("No active request; cancel acknowledged, no runtime reset needed.")
            self.cancel_requested.clear()
            return

        with self.runtime_reset_lock:
            self._reset_runtime_locked(reason=f"control_{command.lower()}")
            self.cancel_requested.clear()

    # -----------------------------------------------------------------------
    # Config / parameter helpers
    # -----------------------------------------------------------------------

    def _param_str(self, name: str) -> str:
        value = self.get_parameter(name).value
        return value.strip() if isinstance(value, str) else ""

    def _load_config(self) -> dict:
        config_path = self._param_str("config_path")
        if not config_path:
            return {}
        with open(os.path.expanduser(config_path), "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    @staticmethod
    def _parse_bool(value, default: bool) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "y", "on"}:
                return True
            if normalized in {"0", "false", "no", "n", "off"}:
                return False
        return default

    def _resolve_inline_system_prompt(self, config: dict) -> bool:
        """
        Whether to inline the system prompt into every runtime prompt string
        rather than injecting it via the SYSTEM_PROMPT env var at startup.
        Resolved from ROS param first, then config YAML, defaulting to False.
        """
        param_raw = self._param_str("inline_system_prompt")
        if param_raw:
            return self._parse_bool(param_raw, default=False)
        if "inline_system_prompt" in config:
            return self._parse_bool(config["inline_system_prompt"], default=False)
        return False

    # -----------------------------------------------------------------------
    # Prompt building
    # Two paths: plain-text fallback and structured envelope (preferred).
    # Both append the same JSON output contract so the model knows what shape
    # of response to produce.
    # -----------------------------------------------------------------------

    # Injected at the end of every prompt so the model knows the response contract
    _JSON_CONTRACT = (
        "Return exactly one JSON object and nothing else. "
        "Required keys: thoughts (string), tool_calls (array), final_output (string). "
        'If speaking is appropriate, include tool_calls like '
        '[{"say_phrase": {"msg": "..."}}].'
    )

    @staticmethod
    def _to_single_line(text: str) -> str:
        """Collapse all whitespace/newlines into single spaces (safe for stdin prompts)."""
        return " ".join(text.replace("\r", "\n").split())

    def _build_runtime_prompt(self, user_prompt: str) -> str:
        """Build a prompt from plain-text input (fallback path)."""
        compact = self._to_single_line(user_prompt)
        if not self.inline_system_prompt or not self.system_prompt:
            return f"User request: {compact} {self._JSON_CONTRACT}"
        compact_sys = self._to_single_line(self.system_prompt)
        return f"System prompt: {compact_sys} User request: {compact} {self._JSON_CONTRACT}"

    def _build_runtime_prompt_from_envelope(self, envelope: dict) -> str:
        """Build a prompt from a structured input envelope (preferred path)."""
        summary_parts = [
            f"channel={envelope['channel']}",
            f"source={envelope['source']}",
            f"type={envelope['type']}",
            f"event={envelope['event'] or 'none'}",
            f"urgency={envelope['urgency']}",
            f"ts={envelope['ts']:.3f}",
        ]
        content_parts = ["Structured input envelope: " + ", ".join(summary_parts)]
        if envelope["text"]:
            content_parts.append(f"text={self._to_single_line(envelope['text'])}")
        if envelope["payload"]:
            compact_payload = json.dumps(envelope["payload"], sort_keys=True, separators=(",", ":"))
            content_parts.append(f"payload={compact_payload}")

        body = " ".join(content_parts)

        if not self.inline_system_prompt or not self.system_prompt:
            return f"{body} {self._JSON_CONTRACT}"
        compact_sys = self._to_single_line(self.system_prompt)
        return f"System prompt: {compact_sys} {body} {self._JSON_CONTRACT}"

    # -----------------------------------------------------------------------
    # Envelope parsing
    # Validates the JSON envelope structure published by skull_control_bt_node.
    # Returns None if the message isn't a valid envelope (triggers plaintext fallback).
    # -----------------------------------------------------------------------

    def _parse_input_envelope(self, raw_input: str):
        """
        Deserialise and validate a structured llm_input envelope.
        Returns a normalised dict on success, or None if the input is not
        a valid envelope (allowing plain-text fallback behaviour).
        """
        try:
            data = json.loads(raw_input)
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

        envelope = {
            "channel": channel,
            "source": source.strip(),
            "type": msg_type.strip(),
            "event": str(data.get("event", "") or "").strip(),
            "urgency": urgency,
            "ts": float(ts),
            "text": "",
            "payload": {},
        }

        text = data.get("text")
        if isinstance(text, str):
            envelope["text"] = text.strip()

        payload = data.get("payload")
        if isinstance(payload, dict):
            envelope["payload"] = payload

        return envelope

    # -----------------------------------------------------------------------
    # Runtime output helpers
    # -----------------------------------------------------------------------

    def _clean_runtime_output_for_logs(self, raw_output: str) -> str:
        """Strip ANSI codes and runtime log lines ([D][, [I][, etc.) for clean logging."""
        lines: list[str] = []
        for line in raw_output.splitlines():
            stripped = self._ANSI_ESCAPE_PATTERN.sub("", line).strip()
            if not stripped:
                continue
            if self._RUNTIME_LOG_LINE_PATTERN.match(stripped):
                continue
            lines.append(stripped)
        return "\n".join(lines)

    def _has_runtime_error(self, raw_output: str) -> bool:
        """Detect known KV-cache / context-full error strings in runtime output."""
        normalized = self._ANSI_ESCAPE_PATTERN.sub("", raw_output).lower()
        return (
            "setkvcache failed" in normalized
            or "context may be full" in normalized
            or ("precompute_len" in normalized and "prefill_max_kv_cache" in normalized)
        )

    def _runtime_context_full(self, raw_output: str) -> bool:
        """True if the runtime reported the context window is full."""
        normalized = self._ANSI_ESCAPE_PATTERN.sub("", raw_output).lower()
        return "context may be full" in normalized

    # -----------------------------------------------------------------------
    # Main prompt callback
    # -----------------------------------------------------------------------

    def prompt_callback(self, msg: String) -> None:
        """
        Receive an input from /skull_control/llm_input, run inference, and
        publish any say_phrase tool calls to /text_to_speech/text_input.

        Flow:
          1. Parse as structured envelope; fall back to plain text.
          2. Build runtime prompt string.
          3. Run inference (blocking; guarded by runtime_reset_lock).
          4. Check for runtime errors; reset runtime if context is full.
          5. Parse model output for say_phrase tool calls.
          6. For human-channel inputs with no phrase, force at least one reply.
          7. Publish each phrase, checking for cancellation between phrases.
        """
        raw_input = msg.data.strip()
        if not raw_input:
            self.get_logger().warning("Received empty llm_input; ignoring.")
            return

        envelope = self._parse_input_envelope(raw_input)
        if envelope is None:
            self.get_logger().info(f"llm_input plain-text fallback: {raw_input}")
        else:
            self.get_logger().info(
                f"llm_input envelope — channel={envelope['channel']} "
                f"type={envelope['type']} event={envelope['event'] or 'none'} "
                f"source={envelope['source']}"
            )

        try:
            self.request_in_flight.set()

            with self.runtime_reset_lock:
                if envelope is None:
                    runtime_prompt = self._build_runtime_prompt(raw_input)
                else:
                    runtime_prompt = self._build_runtime_prompt_from_envelope(envelope)
                raw_output = self.runtime_client.generate(runtime_prompt)

            cleaned = self._clean_runtime_output_for_logs(raw_output)
            self.get_logger().debug(f"Raw runtime output: {raw_output}")
            self.get_logger().info(f"Model output: {cleaned or '<empty>'}")

            # Bail out on known runtime error patterns
            if self._has_runtime_error(raw_output):
                self.get_logger().warning("Runtime error in model output; suppressing publish.")
                if self._runtime_context_full(raw_output):
                    with self.runtime_reset_lock:
                        self._reset_runtime_locked(reason="context_full")
                return

            parsed = parse_response(raw_output)
            if parsed is None:
                self.get_logger().error("Failed to parse model output into the expected JSON contract.")
                return

            # Don't bother publishing if cancelled between inference and publish
            if self.cancel_requested.is_set():
                self.get_logger().warning("LLM response cancelled before publish; dropping output.")
                return

            phrases = extract_say_phrase_calls(parsed)

            # Human input must always produce at least one spoken reply;
            # fall back to final_output text, or a generic acknowledgement.
            if envelope is not None and envelope.get("channel") == "human" and not phrases:
                final_output = parsed.get("final_output")
                fallback = (
                    final_output.strip()
                    if isinstance(final_output, str) and final_output.strip()
                    else "Acknowledged."
                )
                phrases = [fallback]
                self.get_logger().warning(
                    "Human input returned without say_phrase; using final_output fallback."
                )

            for phrase in phrases:
                if self.cancel_requested.is_set():
                    self.get_logger().warning("Cancel received during publish loop; stopping.")
                    break
                tts_msg = String()
                tts_msg.data = phrase
                self.tts_pub.publish(tts_msg)
                self.get_logger().info(f"Published TTS phrase: {phrase}")

            final_output = parsed.get("final_output")
            if isinstance(final_output, str) and final_output.strip():
                self.get_logger().info(f"Final output: {final_output.strip()}")

        except Exception as error:
            if self.cancel_requested.is_set():
                self.get_logger().warning(f"Prompt processing cancelled mid-inference: {error}")
            else:
                self.get_logger().error(f"Error processing prompt: {error}")
        finally:
            # Always clear flags on exit so the next request starts clean
            self.request_in_flight.clear()
            if self.cancel_requested.is_set():
                self.cancel_requested.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LLMAgentAxclNode()
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