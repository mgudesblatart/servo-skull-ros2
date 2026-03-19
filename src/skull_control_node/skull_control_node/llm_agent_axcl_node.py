#!/usr/bin/env python3
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
    _RUNTIME_LOG_LINE_PATTERN = re.compile(r"^\[[A-Z]\]\[")
    _ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")
    _VALID_CHANNELS = {"human", "system", "mixed"}
    _VALID_URGENCIES = {"low", "medium", "high"}

    def __init__(self) -> None:
        super().__init__("llm_agent_axcl_node")
        self.callback_group = ReentrantCallbackGroup()
        self.cancel_requested = threading.Event()
        self.request_in_flight = threading.Event()
        self.runtime_reset_lock = threading.RLock()

        self.declare_parameter("config_path", "")
        self.declare_parameter("runtime_command", "")
        self.declare_parameter("runtime_cwd", "")
        self.declare_parameter("prompt_marker", "prompt >>")
        self.declare_parameter("startup_timeout_sec", 120.0)
        self.declare_parameter("response_timeout_sec", 60.0)
        self.declare_parameter("inline_system_prompt", "")

        config = self._load_config()

        runtime_command = self._param_str("runtime_command") or config.get("runtime_command", "")
        runtime_cwd = self._param_str("runtime_cwd") or config.get("runtime_cwd", "")
        prompt_marker = self._param_str("prompt_marker") or config.get("prompt_marker", "prompt >>")
        startup_timeout_sec = float(self.get_parameter("startup_timeout_sec").value or config.get("startup_timeout_sec", 120.0))
        response_timeout_sec = float(self.get_parameter("response_timeout_sec").value or config.get("response_timeout_sec", 60.0))
        self.inline_system_prompt = self._resolve_inline_system_prompt(config)

        self.system_prompt = config.get("system_prompt", "").strip()
        runtime_env_overrides: dict[str, str] = {}

        if not self.inline_system_prompt and self.system_prompt:
            existing_system_prompt = os.environ.get("SYSTEM_PROMPT", "").strip()
            if existing_system_prompt:
                self.get_logger().info("Using SYSTEM_PROMPT from environment for runtime startup prompt.")
            else:
                runtime_env_overrides["SYSTEM_PROMPT"] = self.system_prompt
                self.get_logger().info("Using system_prompt from config YAML for runtime startup prompt.")

        if not runtime_command:
            raise RuntimeError("runtime_command is not configured. Set it via ROS param or axcl_servo_skull.yaml.")

        self.runtime_client = AxclRuntimeClient(
            command=os.path.expanduser(runtime_command),
            cwd=os.path.expanduser(runtime_cwd) if runtime_cwd else None,
            env_overrides=runtime_env_overrides,
            prompt_marker=prompt_marker,
            startup_timeout_sec=startup_timeout_sec,
            response_timeout_sec=response_timeout_sec,
        )

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
        self.tts_pub = self.create_publisher(String, "/text_to_speech/text_input", 10)

        self.get_logger().info("Starting AXCL runtime client...")
        self.runtime_client.start()
        self.get_logger().info("LLM Agent AXCL Node started.")

    def _reset_runtime_locked(self, reason: str) -> None:
        # Caller must hold runtime_reset_lock.
        try:
            self.runtime_client.close()
        except Exception as error:
            self.get_logger().warning(f"Runtime close during {reason} raised: {error}")

        try:
            self.runtime_client.start()
            self.get_logger().info(f"AXCL runtime restarted ({reason}).")
        except Exception as error:
            self.get_logger().error(f"Failed to restart AXCL runtime ({reason}): {error}")

    def control_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command not in {"CANCEL", "HALT", "STOP"}:
            return

        self.cancel_requested.set()
        self.get_logger().warning(f"Received LLM control command: {command}. Cancelling active request.")

        if not self.request_in_flight.is_set():
            self.get_logger().info("No active LLM request; skipping runtime reset.")
            self.cancel_requested.clear()
            return

        with self.runtime_reset_lock:
            self._reset_runtime_locked(reason=f"control_{command.lower()}")
            self.cancel_requested.clear()

    def _param_str(self, name: str) -> str:
        value = self.get_parameter(name).value
        return value.strip() if isinstance(value, str) else ""

    def _load_config(self) -> dict:
        config_path = self._param_str("config_path")
        if not config_path:
            return {}

        expanded = os.path.expanduser(config_path)
        with open(expanded, "r", encoding="utf-8") as config_file:
            return yaml.safe_load(config_file) or {}

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
        param_raw = self._param_str("inline_system_prompt")
        if param_raw:
            return self._parse_bool(param_raw, default=False)

        if "inline_system_prompt" in config:
            return self._parse_bool(config.get("inline_system_prompt"), default=False)

        return False

    def _build_runtime_prompt(self, user_prompt: str) -> str:
        def to_single_line(text: str) -> str:
            return " ".join(text.replace("\r", "\n").split())

        json_contract = (
            "Return exactly one JSON object and nothing else. "
            "Required keys: thoughts (string), tool_calls (array), final_output (string). "
            "If speaking is appropriate, include tool_calls like "
            "[{\"say_phrase\": {\"msg\": \"...\"}}]."
        )
        compact_user_prompt = to_single_line(user_prompt)

        if not self.inline_system_prompt or not self.system_prompt:
            return f"User request: {compact_user_prompt} {json_contract}"

        compact_system_prompt = to_single_line(self.system_prompt)
        return f"System prompt: {compact_system_prompt} User request: {compact_user_prompt} {json_contract}"

    def _parse_input_envelope(self, raw_input: str):
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

    def _build_runtime_prompt_from_envelope(self, envelope: dict) -> str:
        def to_single_line(text: str) -> str:
            return " ".join(text.replace("\r", "\n").split())

        json_contract = (
            "Return exactly one JSON object and nothing else. "
            "Required keys: thoughts (string), tool_calls (array), final_output (string). "
            "If speaking is appropriate, include tool_calls like "
            "[{\"say_phrase\": {\"msg\": \"...\"}}]."
        )

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
            content_parts.append(f"text={to_single_line(envelope['text'])}")
        if envelope["payload"]:
            compact_payload = json.dumps(envelope["payload"], sort_keys=True, separators=(",", ":"))
            content_parts.append(f"payload={compact_payload}")

        body = " ".join(content_parts)

        if not self.inline_system_prompt or not self.system_prompt:
            return f"{body} {json_contract}"

        compact_system_prompt = to_single_line(self.system_prompt)
        return f"System prompt: {compact_system_prompt} {body} {json_contract}"

    def _clean_runtime_output_for_logs(self, raw_output: str) -> str:
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
        normalized = self._ANSI_ESCAPE_PATTERN.sub("", raw_output).lower()
        return (
            "setkvcache failed" in normalized
            or "context may be full" in normalized
            or "precompute_len" in normalized and "prefill_max_kv_cache" in normalized
        )

    def _runtime_context_full(self, raw_output: str) -> bool:
        normalized = self._ANSI_ESCAPE_PATTERN.sub("", raw_output).lower()
        return "context may be full" in normalized

    def prompt_callback(self, msg: String) -> None:
        raw_input = msg.data.strip()
        if not raw_input:
            self.get_logger().warning("Received empty transcript; ignoring.")
            return

        envelope = self._parse_input_envelope(raw_input)
        if envelope is None:
            self.get_logger().info(f"Received plain-text llm_input (fallback): {raw_input}")
        else:
            self.get_logger().info(
                "Received llm_input envelope "
                f"channel={envelope['channel']} type={envelope['type']} "
                f"event={envelope['event'] or 'none'} source={envelope['source']}"
            )

        try:
            self.request_in_flight.set()
            with self.runtime_reset_lock:
                if envelope is None:
                    runtime_prompt = self._build_runtime_prompt(raw_input)
                else:
                    runtime_prompt = self._build_runtime_prompt_from_envelope(envelope)
                raw_output = self.runtime_client.generate(runtime_prompt)
            cleaned_output = self._clean_runtime_output_for_logs(raw_output)
            self.get_logger().debug(f"Raw runtime output: {raw_output}")
            self.get_logger().info(f"Model output: {cleaned_output or '<empty>'}")

            if self._has_runtime_error(raw_output):
                self.get_logger().warning("Runtime diagnostics detected in model output; suppressing response publish.")
                if self._runtime_context_full(raw_output):
                    with self.runtime_reset_lock:
                        self._reset_runtime_locked(reason="context_full")
                return

            parsed = parse_response(raw_output)
            if parsed is None:
                self.get_logger().error("Failed to parse runtime output into the expected JSON contract.")
                return

            if self.cancel_requested.is_set():
                self.get_logger().warning("LLM response was cancelled before publish; dropping output.")
                return

            phrases = extract_say_phrase_calls(parsed)
            if envelope is not None and envelope.get("channel") == "human" and not phrases:
                fallback_phrase = ""
                final_output = parsed.get("final_output")
                if isinstance(final_output, str) and final_output.strip():
                    fallback_phrase = final_output.strip()
                if not fallback_phrase:
                    fallback_phrase = "Acknowledged."
                phrases = [fallback_phrase]
                self.get_logger().warning(
                    "Human input returned without say_phrase; forcing one spoken reply from final_output fallback."
                )

            for phrase in phrases:
                if self.cancel_requested.is_set():
                    self.get_logger().warning("LLM cancel received during publish loop; stopping phrase publication.")
                    break
                tts_msg = String()
                tts_msg.data = phrase
                self.tts_pub.publish(tts_msg)
                self.get_logger().info(f"Published TTS phrase: {phrase}")

            final_output = parsed.get("final_output")
            if isinstance(final_output, str) and final_output.strip():
                self.get_logger().info(f"Final output: {final_output}")

        except Exception as error:
            if self.cancel_requested.is_set():
                self.get_logger().warning(f"Prompt processing cancelled: {error}")
            else:
                self.get_logger().error(f"Error processing prompt: {error}")
        finally:
            # If a cancel arrived while processing and control callback did not clear it,
            # avoid carrying stale cancel state into the next request.
            self.request_in_flight.clear()
            if self.cancel_requested.is_set():
                self.cancel_requested.clear()

    def destroy_node(self) -> bool:
        with self.runtime_reset_lock:
            self.runtime_client.close()
        return super().destroy_node()


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