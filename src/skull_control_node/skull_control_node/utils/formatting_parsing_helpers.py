import json
import re

ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")
WHITESPACE_PATTERN = re.compile(r"\s+")


def compact_detail(text: str, max_chars: int = 180) -> str:
    cleaned = re.sub(r"\s+", " ", (text or "").strip())
    if len(cleaned) <= max_chars:
        return cleaned
    return cleaned[: max_chars - 3].rstrip() + "..."


def strip_boilerplate_from_phrase(phrase: str, boilerplate_patterns: tuple[str, ...]) -> str:
    text = str(phrase or "").strip()
    for pattern in boilerplate_patterns:
        text = re.sub(pattern, "", text, flags=re.IGNORECASE)
    text = re.sub(r"\s*[.,;]\s*$", "", text.strip())
    text = re.sub(r"\s+", " ", text).strip()
    return text


def sanitize_spoken_phrase(
    phrase: str,
    blocked_speech_markers: tuple[str, ...],
    boilerplate_patterns: tuple[str, ...],
) -> str:
    """Reject JSON/log contamination; clean whitespace; strip boilerplate."""
    cleaned = ANSI_ESCAPE_PATTERN.sub("", phrase or "")
    cleaned = cleaned.strip().strip('"').strip("'")
    cleaned = WHITESPACE_PATTERN.sub(" ", cleaned).strip()
    if not cleaned:
        return ""
    lowered = cleaned.lower()
    if any(marker in lowered for marker in blocked_speech_markers):
        return ""
    if "{" in cleaned or "}" in cleaned:
        return ""
    return strip_boilerplate_from_phrase(cleaned, boilerplate_patterns)


def normalize_assistant_history_contract(
    parsed: dict,
    spoken_phrases: list[str],
    boilerplate_patterns: tuple[str, ...],
) -> str:
    """Persist assistant turns in strict contract shape for stable history."""
    thoughts = parsed.get("thoughts")
    thoughts_text = compact_detail(thoughts, max_chars=160) if isinstance(thoughts, str) else ""

    tool_calls: list[dict] = []
    if spoken_phrases:
        phrase = compact_detail(spoken_phrases[0], max_chars=180)
        phrase = strip_boilerplate_from_phrase(phrase, boilerplate_patterns)
        if phrase:
            tool_calls = [{"say_phrase": {"msg": phrase}}]
    elif isinstance(parsed.get("tool_calls"), list):
        for call in parsed["tool_calls"]:
            if not isinstance(call, dict):
                continue
            say_phrase = call.get("say_phrase")
            if not isinstance(say_phrase, dict):
                continue
            msg = say_phrase.get("msg")
            if not isinstance(msg, str):
                continue
            compact_msg = compact_detail(msg, max_chars=180)
            compact_msg = strip_boilerplate_from_phrase(compact_msg, boilerplate_patterns)
            if compact_msg:
                tool_calls.append({"say_phrase": {"msg": compact_msg}})
                break

    contract_obj = {
        "thoughts": thoughts_text,
        "tool_calls": tool_calls,
    }
    return json.dumps(contract_obj, separators=(",", ":"), ensure_ascii=False)
