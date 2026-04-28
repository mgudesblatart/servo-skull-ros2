from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class ProcessedResponseArtifacts:
    parsed: dict[str, Any]
    phrases: list[str]
    history_user_content: str
    compact_assistant_text: str
    raw_assistant_text: str


def process_model_response(
    *,
    raw_response: str,
    envelope: dict[str, Any],
    user_content: str,
    parse_response_fn: Callable[[str], dict[str, Any] | None],
    extract_say_phrase_calls_fn: Callable[[dict[str, Any]], list[str]],
    sanitize_spoken_phrase_fn: Callable[[str], str],
    build_history_user_content_fn,
    compact_detail_fn,
    normalize_assistant_history_contract_fn,
    boilerplate_patterns: tuple[str, ...],
    blocked_speech_markers: tuple[str, ...],
    logger,
    publish_status,
) -> ProcessedResponseArtifacts | None:
    parsed = parse_response_fn(raw_response)
    if parsed is None:
        logger.error("Failed to parse model response into JSON contract.")
        publish_status(
            "parse_error",
            reason="invalid_model_output",
            detail="Response failed JSON-contract parsing.",
        )
        return None

    phrases = [
        p
        for p in (
            sanitize_spoken_phrase_fn(
                p,
                blocked_speech_markers=blocked_speech_markers,
                boilerplate_patterns=boilerplate_patterns,
            )
            for p in extract_say_phrase_calls_fn(parsed)
        )
        if p
    ]

    history_user_content = build_history_user_content_fn(
        envelope,
        user_content,
        compact_detail_fn=compact_detail_fn,
    )
    compact_assistant_text = normalize_assistant_history_contract_fn(
        parsed,
        phrases,
        boilerplate_patterns=boilerplate_patterns,
    )

    raw_assistant_text = raw_response if isinstance(raw_response, str) else ""
    if not raw_assistant_text.strip():
        raw_assistant_text = compact_assistant_text

    return ProcessedResponseArtifacts(
        parsed=parsed,
        phrases=phrases,
        history_user_content=history_user_content,
        compact_assistant_text=compact_assistant_text,
        raw_assistant_text=raw_assistant_text,
    )
