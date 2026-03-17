import json
import re
from typing import Any


REQUIRED_KEYS = {"thoughts", "tool_calls", "final_output"}
RUNTIME_LOG_LINE_PATTERN = re.compile(r"^\[[A-Z]\]\[")


def _strip_markdown_fences(raw_text: str) -> str:
    text = raw_text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    return text


def _extract_balanced_json_block(raw_text: str) -> str | None:
    start = raw_text.find("{")
    if start == -1:
        return None

    depth = 0
    in_string = False
    escape = False

    for index in range(start, len(raw_text)):
        char = raw_text[index]

        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return raw_text[start:index + 1]

    return None


def _validate_schema(payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    if not REQUIRED_KEYS.issubset(payload.keys()):
        return None
    if not isinstance(payload["tool_calls"], list):
        return None
    return payload


def _extract_plaintext_reply(raw_text: str) -> str:
    lines: list[str] = []

    for line in raw_text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped == "prompt >>":
            continue
        if RUNTIME_LOG_LINE_PATTERN.match(stripped):
            continue
        lines.append(stripped)

    if not lines:
        return ""

    return " ".join(lines).strip()


def _build_fallback_contract(raw_text: str) -> dict[str, Any] | None:
    fallback_text = _extract_plaintext_reply(raw_text)
    if not fallback_text:
        return None

    return {
        "thoughts": "Runtime returned non-JSON text; wrapping in fallback contract.",
        "tool_calls": [{"say_phrase": {"msg": fallback_text}}],
        "final_output": fallback_text,
    }


def parse_response(raw_text: str) -> dict[str, Any] | None:
    candidates: list[str] = []

    stripped = _strip_markdown_fences(raw_text)
    if stripped:
        candidates.append(stripped)

    balanced = _extract_balanced_json_block(stripped or raw_text)
    if balanced and balanced not in candidates:
        candidates.append(balanced)

    regex_match = re.search(r"\{.*\}", raw_text, re.DOTALL)
    if regex_match:
        candidate = regex_match.group(0).strip()
        if candidate not in candidates:
            candidates.append(candidate)

    for candidate in candidates:
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        valid = _validate_schema(parsed)
        if valid is not None:
            return valid

    try:
        from json_repair import repair_json
    except ImportError:
        repair_json = None

    if repair_json is not None:
        for candidate in candidates or [raw_text]:
            try:
                repaired = repair_json(candidate, return_objects=True)
            except Exception:
                continue
            valid = _validate_schema(repaired)
            if valid is not None:
                return valid

    return _build_fallback_contract(raw_text)


def extract_say_phrase_calls(parsed: dict[str, Any]) -> list[str]:
    messages: list[str] = []

    for tool_call in parsed.get("tool_calls", []):
        if not isinstance(tool_call, dict):
            continue

        say_phrase = tool_call.get("say_phrase")
        if isinstance(say_phrase, dict):
            message = say_phrase.get("msg") or say_phrase.get("text")
            if isinstance(message, str) and message.strip():
                messages.append(message.strip())
            continue

        if tool_call.get("name") == "say_phrase":
            arguments = tool_call.get("args", {})
            if isinstance(arguments, dict):
                message = arguments.get("msg") or arguments.get("text")
                if isinstance(message, str) and message.strip():
                    messages.append(message.strip())

    return messages