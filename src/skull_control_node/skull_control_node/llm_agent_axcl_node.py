#!/usr/bin/env python3
import os
import re

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from skull_control_node.axcl_runtime_client import AxclRuntimeClient
from skull_control_node.response_parser import extract_say_phrase_calls, parse_response


class LLMAgentAxclNode(Node):
    _RUNTIME_LOG_LINE_PATTERN = re.compile(r"^\[[A-Z]\]\[")

    def __init__(self) -> None:
        super().__init__("llm_agent_axcl_node")

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
            "/speech_to_text/transcript",
            self.prompt_callback,
            10,
        )
        self.tts_pub = self.create_publisher(String, "/text_to_speech/text_input", 10)

        self.get_logger().info("Starting AXCL runtime client...")
        self.runtime_client.start()
        self.get_logger().info("LLM Agent AXCL Node started.")

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

    def _clean_runtime_output_for_logs(self, raw_output: str) -> str:
        lines: list[str] = []
        for line in raw_output.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            if self._RUNTIME_LOG_LINE_PATTERN.match(stripped):
                continue
            lines.append(stripped)
        return "\n".join(lines)

    def prompt_callback(self, msg: String) -> None:
        user_prompt = msg.data.strip()
        if not user_prompt:
            self.get_logger().warning("Received empty transcript; ignoring.")
            return

        self.get_logger().info(f"Received prompt: {user_prompt}")

        try:
            runtime_prompt = self._build_runtime_prompt(user_prompt)
            raw_output = self.runtime_client.generate(runtime_prompt)
            cleaned_output = self._clean_runtime_output_for_logs(raw_output)
            self.get_logger().debug(f"Raw runtime output: {raw_output}")
            self.get_logger().info(f"Model output: {cleaned_output or '<empty>'}")

            parsed = parse_response(raw_output)
            if parsed is None:
                self.get_logger().error("Failed to parse runtime output into the expected JSON contract.")
                return

            for phrase in extract_say_phrase_calls(parsed):
                tts_msg = String()
                tts_msg.data = phrase
                self.tts_pub.publish(tts_msg)
                self.get_logger().info(f"Published TTS phrase: {phrase}")

            final_output = parsed.get("final_output")
            if isinstance(final_output, str) and final_output.strip():
                self.get_logger().info(f"Final output: {final_output}")

        except Exception as error:
            self.get_logger().error(f"Error processing prompt: {error}")

    def destroy_node(self) -> bool:
        self.runtime_client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LLMAgentAxclNode()
        executor = SingleThreadedExecutor()
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