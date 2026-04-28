import json
import re

from skull_control_node.utils.context_input_helpers import apply_thinking_mode


def format_summary_state_as_text(summary_state: dict) -> str:
    """Format structured summary state as readable text for system prompt seeding."""
    lines = ["Conversation summary:"]

    if summary_state.get("user_preferences"):
        lines.append("User preferences:")
        for pref in summary_state["user_preferences"]:
            lines.append(f"  - {pref}")

    if summary_state.get("active_topics"):
        lines.append("Active topics:")
        for topic in summary_state["active_topics"]:
            lines.append(f"  - {topic}")

    if summary_state.get("open_loops"):
        lines.append("Open questions/loops:")
        for loop in summary_state["open_loops"]:
            lines.append(f"  - {loop}")

    if summary_state.get("assistant_commitments"):
        lines.append("Your prior commitments:")
        for commitment in summary_state["assistant_commitments"]:
            lines.append(f"  - {commitment}")

    if summary_state.get("known_facts"):
        lines.append("Key facts:")
        for fact in summary_state["known_facts"]:
            lines.append(f"  - {fact}")

    return "\n".join(lines)


def build_summary_refinement_messages(
    *,
    prior_summary_state: dict,
    untruncated_turns: list[tuple[str, str]],
    heuristic_summary_state: dict,
    summary_refinement_hint: str,
    default_disable_thinking: bool,
    no_think_excerpt: str,
    disable_thinking: bool,
) -> list[dict]:
    turn_payload = [
        {
            "user": user_content,
            "assistant": assistant_content,
        }
        for user_content, assistant_content in untruncated_turns
    ]
    prompt = {
        "existing_summary": prior_summary_state,
        "recent_turns": turn_payload,
        "heuristic_summary": heuristic_summary_state,
    }
    user_content = json.dumps(prompt, ensure_ascii=False, separators=(",", ":"))
    user_content = apply_thinking_mode(
        user_content,
        default_disable_thinking=default_disable_thinking,
        no_think_excerpt=no_think_excerpt,
        disable_thinking_override=disable_thinking,
    )

    return [
        {
            "role": "system",
            "content": (
                "You compress conversation state for a robot assistant. "
                + summary_refinement_hint
            ),
        },
        {
            "role": "user",
            "content": user_content,
        },
    ]


def extract_first_json_object(raw_response: str) -> dict | None:
    """Extract first top-level JSON object from model text, tolerating think noise."""
    sanitized = re.sub(
        r"<think>.*?</think>",
        "",
        raw_response or "",
        flags=re.DOTALL | re.IGNORECASE,
    )
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


def parse_summary_refinement_response(
    raw_response: str,
    *,
    normalize_summary_state_fn,
    summary_fields: tuple[str, ...],
) -> dict | None:
    parsed = extract_first_json_object(raw_response)
    if parsed is None:
        return None

    normalized = normalize_summary_state_fn(parsed)
    if not any(normalized[field] for field in summary_fields):
        return None
    return normalized
