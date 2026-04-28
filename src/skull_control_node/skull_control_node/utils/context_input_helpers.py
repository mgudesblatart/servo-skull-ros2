import json


def estimate_message_tokens(messages: list[dict]) -> int:
    total = 0
    for message in messages:
        content = str(message.get("content", ""))
        total += max(1, int(len(content) / 4)) + 4
    return total


def parse_input_envelope(
    msg,
    *,
    valid_channels: set[str],
    valid_urgencies: set[str],
) -> dict | None:
    channel = str(msg.channel or "").strip().lower()
    source = str(msg.source or "").strip()
    msg_type = str(msg.type or "").strip().lower()
    urgency = str(msg.urgency or "").strip().lower()
    event = str(msg.event or "").strip().lower()
    ts = msg.ts

    if channel not in valid_channels:
        if event == "human_transcript" and msg_type == "transcript" and source.lower() == "stt":
            channel = "human"
        else:
            return None
    if not source:
        return None
    if not msg_type:
        return None
    if urgency not in valid_urgencies:
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


def build_user_content_raw(envelope: dict, json_contract_hint: str) -> str:
    """Format envelope into a compact user-turn payload."""
    if envelope["text"] and envelope["channel"] == "human":
        return (
            "Input Class: DIRECT_HUMAN\n"
            f"User transcript: {envelope['text']}\n\n"
            f"{json_contract_hint}"
        )

    system_request = envelope["text"] or (envelope.get("event") or "event_update")
    return (
        "Input Class: SYSTEM_EVENT\n"
        f"System Request: {system_request}\n\n"
        f"{json_contract_hint}"
    )


def apply_thinking_mode(
    content: str,
    *,
    default_disable_thinking: bool,
    no_think_excerpt: str,
    disable_thinking_override: bool | None = None,
) -> str:
    text = (content or "").strip()
    should_disable = default_disable_thinking if disable_thinking_override is None else bool(disable_thinking_override)
    if not should_disable:
        return text
    if no_think_excerpt in text:
        return text
    return f"{text}\n\n{no_think_excerpt}".strip()


def build_history_user_content(envelope: dict, prompt_user_content: str, compact_detail_fn) -> str:
    """Store compact user-turn text without repeated policy scaffolding."""
    channel = str(envelope.get("channel") or "").strip().lower()
    text = envelope.get("text")
    event = str(envelope.get("event") or "").strip().lower()

    if channel == "human" and isinstance(text, str) and text.strip():
        return compact_detail_fn(text, max_chars=220)

    if isinstance(text, str) and text.strip():
        return compact_detail_fn(f"{event or 'event'}: {text}", max_chars=220)

    fallback = prompt_user_content or event or "event_update"
    return compact_detail_fn(fallback, max_chars=220)


def extract_untruncated_turns_from_append_history(append_history: list[dict]) -> list[tuple[str, str]]:
    """Extract user/assistant pairs from append history, skipping system messages."""
    turns: list[tuple[str, str]] = []
    i = 0
    while i < len(append_history) and append_history[i].get("role") == "system":
        i += 1

    while i < len(append_history) - 1:
        user_msg = append_history[i]
        assistant_msg = append_history[i + 1]
        if user_msg.get("role") == "user" and assistant_msg.get("role") == "assistant":
            turns.append((user_msg.get("content", ""), assistant_msg.get("content", "")))
            i += 2
        else:
            i += 1
    return turns
