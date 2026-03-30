"""
response_parser.py

Parses raw text output from the LLM runtime into the expected JSON contract:

    {
        "thoughts": "...",
        "tool_calls": [{"say_phrase": {"msg": "..."}}]
    }

The model doesn't always return clean JSON, so we try several recovery
strategies in order before giving up and wrapping the whole output in a
synthetic fallback contract (so something is always spoken rather than silence).

Parsing pipeline (parse_response):
  1. Strip markdown code fences if present.
  2. Try to extract a balanced {} block with a character-walking scanner.
  3. Try a greedy regex match for {.*}.
  4. json.loads each candidate; validate required keys.
  5. If json_repair is available, try it on all candidates.
  6. Fall back to wrapping the whole cleaned text in a synthetic contract.
"""

import json
import re
from typing import Any


# Only keys the model contract permits.
REQUIRED_KEYS = {"thoughts", "tool_calls"}

# Used to identify and skip runtime diagnostic lines in model output
RUNTIME_LOG_LINE_PATTERN = re.compile(r"^\[[A-Z]\]\[")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[[0-9;]*m")


# ---------------------------------------------------------------------------
# Text cleaning helpers
# ---------------------------------------------------------------------------

def _strip_ansi(text: str) -> str:
    return ANSI_ESCAPE_PATTERN.sub("", text)


def _strip_markdown_fences(raw_text: str) -> str:
    """Remove ```json ... ``` or ``` ... ``` wrappers if the model added them."""
    text = raw_text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    return text


# ---------------------------------------------------------------------------
# JSON typo repair
# ---------------------------------------------------------------------------

def _repair_common_json_typos(text: str) -> str:
    """
    Fix recurring model output typos that break valid JSON parsing.
    Handles:
      - "key",{...} -> "key":{...}
      - "key","value" -> "key":"value"
    but only when the malformed key appears in an object entry position.
    """
    repaired = re.sub(
        r'([,{]\s*)"([^"]+)",\s*(\{)',
        r'\1"\2": \3',
        text,
    )
    repaired = re.sub(
        r'([,{]\s*)"([^"]+)",\s*"((?:\\.|[^"\\])*)"',
        r'\1"\2": "\3"',
        repaired,
    )
    return repaired


# ---------------------------------------------------------------------------
# JSON extraction strategies
# ---------------------------------------------------------------------------

def _extract_balanced_json_block(raw_text: str) -> str | None:
    """
    Walk the string character by character to find the first complete, balanced
    JSON object. More reliable than regex for nested structures.
    Returns the raw JSON substring, or None if no balanced block is found.
    """
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
    """Return normalized payload if it satisfies the strict contract, else None."""
    if not isinstance(payload, dict):
        return None
    if not REQUIRED_KEYS.issubset(payload.keys()):
        return None
    if not isinstance(payload["thoughts"], str):
        return None
    if not isinstance(payload["tool_calls"], list):
        return None
    # Normalize to the strict two-key contract to avoid leaking legacy keys
    # (for example final_output) into downstream state/history.
    return {
        "thoughts": payload["thoughts"],
        "tool_calls": payload["tool_calls"],
    }


# ---------------------------------------------------------------------------
# Fallback: wrap plain text in a synthetic contract so the skull says *something*
# ---------------------------------------------------------------------------

def _extract_plaintext_reply(raw_text: str) -> str:
    """
    Strip all runtime noise and return whatever plain text remains.
    Used as the speech content for the fallback contract.
    """
    lines: list[str] = []
    for line in raw_text.splitlines():
        stripped = _strip_ansi(line).strip()
        if not stripped:
            continue
        if stripped == "prompt >>":
            continue
        if RUNTIME_LOG_LINE_PATTERN.match(stripped):
            continue
        lines.append(stripped)

    return " ".join(lines).strip()


def _build_fallback_contract(raw_text: str) -> dict[str, Any] | None:
    """
    Last resort: if nothing parses, wrap the cleaned plain text in a valid
    contract so the skull at least speaks what the model said verbatim.
    Returns None if there's nothing to say.
    """
    fallback_text = _extract_plaintext_reply(raw_text)
    if not fallback_text:
        return None

    return {
        "thoughts": "Runtime returned non-JSON text; wrapping in fallback contract.",
        "tool_calls": [{"say_phrase": {"msg": fallback_text}}],
    }


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def parse_response(raw_text: str) -> dict[str, Any] | None:
    """
    Try every recovery strategy to extract a valid response contract from
    raw runtime output. Returns a validated dict or None if all strategies fail.
    """
    candidates: list[str] = []

    # Strip Qwen3 <think>...</think> block before any other processing
    think_stripped = re.sub(r"<think>.*?</think>", "", raw_text, flags=re.DOTALL).strip()
    working_text = think_stripped if think_stripped else raw_text
    working_text = _repair_common_json_typos(working_text)

    # Strategy 1: strip markdown fences, try direct parse
    stripped = _strip_markdown_fences(working_text)
    if stripped:
        candidates.append(stripped)

    # Strategy 2: scan for a balanced JSON block
    balanced = _extract_balanced_json_block(stripped or working_text)
    if balanced and balanced not in candidates:
        candidates.append(balanced)

    # Strategy 3: greedy regex {.*} (handles trailing junk after closing brace)
    regex_match = re.search(r"\{.*\}", working_text, re.DOTALL)
    if regex_match:
        candidate = regex_match.group(0).strip()
        if candidate not in candidates:
            candidates.append(candidate)

    # Try json.loads on each candidate
    for candidate in candidates:
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        valid = _validate_schema(parsed)
        if valid is not None:
            return valid

    # Optional: use json_repair if installed (handles truncated/malformed JSON)
    try:
        from json_repair import repair_json
    except ImportError:
        repair_json = None

    if repair_json is not None:
        for candidate in candidates or [working_text]:
            try:
                repaired = repair_json(candidate, return_objects=True)
            except Exception:
                continue
            valid = _validate_schema(repaired)
            if valid is not None:
                return valid

    # Final fallback: wrap whatever the model said in a minimal valid contract
    return _build_fallback_contract(working_text)


def extract_say_phrase_calls(parsed: dict[str, Any]) -> list[str]:
    """
    Extract spoken phrases from the tool_calls array.

    Handles two tool_call formats the model might produce:
      - {"say_phrase": {"msg": "..."}}        (preferred)
      - {"name": "say_phrase", "args": {...}} (alternative)
    """
    messages: list[str] = []

    for tool_call in parsed.get("tool_calls", []):
        if not isinstance(tool_call, dict):
            continue

        # Preferred format: inline dict
        say_phrase = tool_call.get("say_phrase")
        if isinstance(say_phrase, dict):
            message = say_phrase.get("msg") or say_phrase.get("text")
            if isinstance(message, str) and message.strip():
                messages.append(message.strip())
            continue

        # Alternative format: name + args
        if tool_call.get("name") == "say_phrase":
            arguments = tool_call.get("args", {})
            if isinstance(arguments, dict):
                message = arguments.get("msg") or arguments.get("text")
                if isinstance(message, str) and message.strip():
                    messages.append(message.strip())

    return messages