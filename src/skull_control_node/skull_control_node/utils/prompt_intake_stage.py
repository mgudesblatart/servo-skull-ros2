from dataclasses import dataclass
from typing import Any

from skull_control_node.utils.context_input_helpers import (
    apply_thinking_mode,
    build_user_content_raw,
    parse_input_envelope,
)


@dataclass
class PromptIntakeResult:
    accepted: bool
    envelope: dict[str, Any] | None = None
    user_content: str = ""
    reason: str = ""
    log_level: str = "debug"
    log_message: str = ""


def prepare_prompt_intake(
    msg,
    *,
    server_ready: bool,
    valid_channels: set[str],
    valid_urgencies: set[str],
    json_contract_hint: str,
    default_disable_thinking: bool,
    no_think_excerpt: str,
) -> PromptIntakeResult:
    if not msg.channel.strip() and not msg.text.strip():
        return PromptIntakeResult(
            accepted=False,
            reason="empty_input",
            log_level="debug",
            log_message="Dropping empty llm_input envelope.",
        )

    if not server_ready:
        return PromptIntakeResult(
            accepted=False,
            reason="server_not_ready",
            log_level="warning",
            log_message="Prompt received but axllm server not ready yet; dropping.",
        )

    envelope = parse_input_envelope(
        msg,
        valid_channels=valid_channels,
        valid_urgencies=valid_urgencies,
    )
    if envelope is None:
        return PromptIntakeResult(
            accepted=False,
            reason="invalid_envelope",
            log_level="warning",
            log_message="Dropping invalid LlmInput envelope.",
        )

    channel = envelope["channel"]
    if not (
        channel == "human"
        and envelope.get("source") == "stt"
        and envelope.get("type") == "transcript"
        and envelope.get("event") == "human_transcript"
    ):
        return PromptIntakeResult(
            accepted=False,
            reason="minimal_mode_drop",
            log_level="debug",
            log_message=(
                "Dropping non-human/non-STT envelope in minimal mode "
                f"(channel={channel}, event={envelope['event']})."
            ),
            envelope=envelope,
        )

    user_content = build_user_content_raw(envelope, json_contract_hint)
    user_content = apply_thinking_mode(
        user_content,
        default_disable_thinking=default_disable_thinking,
        no_think_excerpt=no_think_excerpt,
    )

    return PromptIntakeResult(
        accepted=True,
        envelope=envelope,
        user_content=user_content,
    )
