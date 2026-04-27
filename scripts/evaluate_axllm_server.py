#!/usr/bin/env python3
"""Quick evaluator for axllm OpenAI-compatible chat endpoints.

Runs the four canned prompt-adherence tests used during HTTP backend bring-up:

1. Human greeting -> should speak
2. Low-urgency system event -> should stay silent
3. High-urgency critical fault -> should speak warning
4. Personality question -> should speak in-character

For each case the script reports:
- Wall-clock latency
- Whether the raw response stripped to strict JSON
- Whether the project response parser accepted it
- Whether tool-call behavior matched expectations

Exit status is non-zero if any test fails.

Cache behavior:
- append (default): keeps one growing message list across cases to favor KV reuse.
- isolated: sends each case as a fresh system+user request.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover - depends on runtime env
    yaml = None


REPO_ROOT = Path(__file__).resolve().parents[1]
SKULL_CONTROL_SRC = REPO_ROOT / "src" / "skull_control_node"
if str(SKULL_CONTROL_SRC) not in sys.path:
    sys.path.insert(0, str(SKULL_CONTROL_SRC))

from skull_control_node.response_parser import extract_say_phrase_calls, parse_response  # noqa: E402


DEFAULT_CONFIG_PATH = REPO_ROOT / "src" / "skull_control_node" / "configs" / "http_servo_skull.yaml"
NODE_JSON_CONTRACT_EXCERPT = (
    "Return exactly one JSON object and nothing else. "
    "Required keys: thoughts (string), tool_calls (array). "
    'If speaking is appropriate, include tool_calls like [{"say_phrase": {"msg": "..."}}].'
)
THINK_BLOCK_PATTERN = re.compile(r"<think>.*?</think>", re.DOTALL | re.IGNORECASE)
THINK_TAG_PATTERN = re.compile(r"</?think>", re.IGNORECASE)
REQUIRED_JSON_KEYS = {"thoughts", "tool_calls"}
NO_THINK_EXCERPT = (
    "Do not include <think> blocks. Return only the final JSON object.\n/no_think"
)


@dataclass(frozen=True)
class TestCase:
    case_id: str
    description: str
    user_content: str
    expect_speech: bool
    expected_phrase_markers: tuple[str, ...] = ()


TEST_CASES: tuple[TestCase, ...] = (
    TestCase(
        case_id="greeting",
        description="Direct human greeting should produce one spoken response.",
        user_content="Good morning. Are you online and listening?",
        expect_speech=True,
    ),
    TestCase(
        case_id="low_system_event",
        description="Low-urgency machine event should remain silent.",
        user_content=(
            "System context: channel=system source=fsm event=tracking_update urgency=low "
            'payload={"tracking":true,"has_target":true,"distance_cm":88}'
        ),
        expect_speech=False,
    ),
    TestCase(
        case_id="critical_fault",
        description="High-urgency critical failure should speak a warning.",
        user_content=(
            "System context: channel=system source=fsm event=critical_fault urgency=high "
            'payload={"fault":"motor_driver_overtemp","severity":"critical","safety":true}'
        ),
        expect_speech=True,
        expected_phrase_markers=("warning", "critical", "attention"),
    ),
    TestCase(
        case_id="personality",
        description="Direct personality question should answer in character.",
        user_content=(
            "What exactly are you, and how would you describe your disposition? "
            "Keep it brief and in character."
        ),
        expect_speech=True,
        expected_phrase_markers=("this unit", "servo skull", "master", "sir"),
    ),
)


def load_yaml_config(config_path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to load config files for this script.")
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def strip_think_blocks(text: str) -> str:
    return THINK_BLOCK_PATTERN.sub("", text).strip()


def sanitize_for_strict_json(text: str) -> str:
    """Drop known think wrappers/tags while preserving remaining payload text."""
    stripped = strip_think_blocks(text)
    stripped = THINK_TAG_PATTERN.sub("", stripped)
    return stripped.strip()


def extract_first_json_object(text: str) -> tuple[dict[str, Any] | None, str]:
    """Return the first decodable top-level JSON object found in text."""
    decoder = json.JSONDecoder()
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            candidate, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict):
            return candidate, ""
    return None, "No top-level JSON object found after sanitization"


def with_json_contract_excerpt(text: str) -> str:
    stripped = text.strip()
    if NODE_JSON_CONTRACT_EXCERPT in stripped:
        return stripped
    return f"{stripped} {NODE_JSON_CONTRACT_EXCERPT}".strip()


def with_no_think_excerpt(text: str) -> str:
    stripped = text.strip()
    if NO_THINK_EXCERPT in stripped:
        return stripped
    return f"{stripped}\n\n{NO_THINK_EXCERPT}".strip()


def request_chat_completion(
    *,
    base_url: str,
    model: str,
    messages: list[dict[str, str]],
    timeout_sec: float,
    max_tokens: int,
) -> tuple[float, str]:
    payload = {
        "model": model,
        "messages": messages,
        "stream": False,
        "max_tokens": max(1, int(max_tokens)),
    }
    request = urllib.request.Request(
        url=f"{base_url.rstrip('/')}/v1/chat/completions",
        method="POST",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
        },
    )

    started = time.perf_counter()
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            raw = response.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as error:
        body = error.read().decode("utf-8", errors="replace") if hasattr(error, "read") else ""
        raise RuntimeError(f"HTTP {error.code}: {body}") from error
    elapsed_sec = time.perf_counter() - started

    payload = json.loads(raw)
    choices = payload.get("choices")
    if not isinstance(choices, list) or not choices:
        raise RuntimeError(f"Response missing choices: {payload}")
    message = choices[0].get("message")
    content = message.get("content") if isinstance(message, dict) else None
    if not isinstance(content, str):
        raise RuntimeError(f"Response missing message.content: {payload}")
    return elapsed_sec, content


def probe_server(base_url: str, timeout_sec: float) -> tuple[bool, str]:
    request = urllib.request.Request(
        url=f"{base_url.rstrip('/')}/v1/models",
        method="GET",
        headers={"Accept": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=min(timeout_sec, 5.0)) as response:
            body = response.read().decode("utf-8", errors="replace")
            if getattr(response, "status", 200) != 200:
                return False, f"/v1/models returned HTTP {getattr(response, 'status', 'unknown')}"
            try:
                json.loads(body)
            except json.JSONDecodeError:
                return False, "/v1/models returned non-JSON payload"
            return True, "ok"
    except Exception as error:
        return False, str(error)


def probe_model(base_url: str, model: str, timeout_sec: float) -> tuple[bool, str]:
    request = urllib.request.Request(
        url=f"{base_url.rstrip('/')}/v1/models",
        method="GET",
        headers={"Accept": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=min(timeout_sec, 5.0)) as response:
            body = response.read().decode("utf-8", errors="replace")
    except Exception as error:
        return False, str(error)

    try:
        payload = json.loads(body)
    except json.JSONDecodeError as error:
        return False, f"Could not parse /v1/models JSON: {error}"

    model_ids: list[str] = []
    if isinstance(payload, dict):
        data = payload.get("data")
        if isinstance(data, list):
            for item in data:
                if isinstance(item, dict):
                    model_id = item.get("id")
                    if isinstance(model_id, str):
                        model_ids.append(model_id)

    if model_ids and model not in model_ids:
        return False, f"Model '{model}' not in available models: {', '.join(model_ids)}"

    return True, "ok"


def reset_server(base_url: str, timeout_sec: float) -> tuple[bool, str]:
    errors: list[str] = []
    for path in ("/reset", "/api/reset"):
        request = urllib.request.Request(
            url=f"{base_url.rstrip('/')}{path}",
            method="POST",
            data=b"{}",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=min(timeout_sec, 5.0)) as response:
                status = getattr(response, "status", 200)
                response.read()
                if status in (200, 204):
                    return True, path
                errors.append(f"{path}: HTTP {status}")
        except Exception as error:
            errors.append(f"{path}: {error}")

    if not errors:
        return False, "no reset endpoints attempted"
    return False, "; ".join(errors)


def evaluate_case(
    *,
    case: TestCase,
    base_url: str,
    model: str,
    messages: list[dict[str, str]],
    timeout_sec: float,
    max_tokens: int,
    dump_raw: bool,
) -> tuple[dict[str, Any], str]:
    elapsed_sec, raw_content = request_chat_completion(
        base_url=base_url,
        model=model,
        messages=messages,
        timeout_sec=timeout_sec,
        max_tokens=max_tokens,
    )

    stripped = sanitize_for_strict_json(raw_content)
    strict_json_ok = False
    strict_json_error = ""
    parsed_strict: dict[str, Any] | None = None
    strict_contract_ok = False
    strict_contract_error = ""
    parsed_candidate, parse_error = extract_first_json_object(stripped)
    if parsed_candidate is not None:
        parsed_strict = parsed_candidate
        strict_json_ok = True
    else:
        strict_json_error = parse_error

    if parsed_strict is not None:
        actual_keys = set(parsed_strict.keys())
        if actual_keys != REQUIRED_JSON_KEYS:
            strict_contract_error = (
                f"Expected keys {sorted(REQUIRED_JSON_KEYS)}, got {sorted(actual_keys)}"
            )
        elif not isinstance(parsed_strict.get("thoughts"), str):
            strict_contract_error = "Field 'thoughts' was not a string"
        elif not isinstance(parsed_strict.get("tool_calls"), list):
            strict_contract_error = "Field 'tool_calls' was not a list"
        else:
            strict_contract_ok = True

    parsed_contract = parse_response(raw_content)
    parser_ok = parsed_contract is not None
    phrases = extract_say_phrase_calls(parsed_contract) if parsed_contract is not None else []
    phrases = [str(phrase).strip() for phrase in phrases if str(phrase).strip()]

    speech_ok = bool(phrases) == case.expect_speech
    marker_ok = True
    if case.expected_phrase_markers:
        joined = " ".join(phrases).lower()
        marker_ok = any(marker in joined for marker in case.expected_phrase_markers)

    passed = strict_json_ok and strict_contract_ok and parser_ok and speech_ok and marker_ok
    result = {
        "case_id": case.case_id,
        "description": case.description,
        "elapsed_sec": elapsed_sec,
        "strict_json_ok": strict_json_ok,
        "strict_json_error": strict_json_error,
        "strict_contract_ok": strict_contract_ok,
        "strict_contract_error": strict_contract_error,
        "parser_ok": parser_ok,
        "expect_speech": case.expect_speech,
        "actual_speech": bool(phrases),
        "speech_ok": speech_ok,
        "marker_ok": marker_ok,
        "phrases": phrases,
        "passed": passed,
    }
    if dump_raw:
        result["raw_content"] = raw_content
        result["stripped_content"] = stripped
        result["strict_json"] = parsed_strict
        result["parser_output"] = parsed_contract
    return result, raw_content


def print_report(results: list[dict[str, Any]]) -> None:
    print()
    print("Case                 Latency   JSON   Contract   Parser   Speech   Markers   Result")
    print("-------------------  --------  -----  ---------  -------  -------  --------  ------")
    for result in results:
        strict_json_ok = bool(result.get("strict_json_ok", False))
        strict_contract_ok = bool(result.get("strict_contract_ok", False))
        parser_ok = bool(result.get("parser_ok", False))
        speech_ok = bool(result.get("speech_ok", False))
        marker_ok = bool(result.get("marker_ok", False))
        phrases = result.get("phrases", []) or []
        print(
            f"{result['case_id']:<19} "
            f"{result['elapsed_sec']:>7.2f}s  "
            f"{('ok' if strict_json_ok else 'fail'):<5}  "
            f"{('ok' if strict_contract_ok else 'fail'):<9}  "
            f"{('ok' if parser_ok else 'fail'):<7}  "
            f"{('ok' if speech_ok else 'fail'):<7}  "
            f"{('ok' if marker_ok else 'n/a' if not phrases else 'fail'):<8}  "
            f"{('PASS' if result['passed'] else 'FAIL')}"
        )
    print()
    for result in results:
        print(f"[{result['case_id']}] {result['description']}")
        phrases = result.get("phrases", []) or []
        if phrases:
            print(f"  phrase: {phrases[0]}")
        else:
            print("  phrase: <silent>")
        strict_json_error = str(result.get("strict_json_error", "") or "")
        strict_contract_error = str(result.get("strict_contract_error", "") or "")
        if not bool(result.get("strict_json_ok", False)) and strict_json_error:
            print(f"  strict-json error: {strict_json_error}")
        if not bool(result.get("strict_contract_ok", False)) and strict_contract_error:
            print(f"  contract error: {strict_contract_error}")
        print(f"  latency: {result['elapsed_sec']:.2f}s")
        print(f"  passed: {result['passed']}")
        print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="", help="axllm server base URL. Defaults to config value or http://127.0.0.1:8081")
    parser.add_argument("--model", default="", help="Model name. Defaults to config value or AXERA-TECH/Qwen3-1.7B")
    parser.add_argument("--config-path", default=str(DEFAULT_CONFIG_PATH), help="YAML config to load model/base URL defaults and system prompt")
    parser.add_argument("--system-prompt-file", default="", help="Optional plain-text file that overrides the system prompt")
    parser.add_argument("--timeout", type=float, default=90.0, help="HTTP timeout per call in seconds")
    parser.add_argument(
        "--max-tokens",
        type=int,
        default=128,
        help="Maximum completion tokens per request (sent as max_tokens).",
    )
    parser.add_argument(
        "--cache-mode",
        choices=("append", "isolated"),
        default="append",
        help="append keeps one growing message list across cases (KV-friendly); isolated sends each case fresh.",
    )
    parser.add_argument(
        "--reset-before-run",
        action="store_true",
        help="Best-effort reset of server-side conversation/KV state via /reset or /api/reset before running cases.",
    )
    parser.add_argument(
        "--disable-thinking",
        action="store_true",
        help="Append a no-think instruction to each user case prompt.",
    )
    parser.add_argument("--only", default="", help="Comma-separated subset of case IDs to run")
    parser.add_argument("--dump-raw", action="store_true", help="Print raw and parsed payloads after the summary")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON instead of a text report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    config: dict[str, Any] = {}
    config_path = Path(args.config_path)
    if args.config_path and config_path.exists():
        config = load_yaml_config(config_path)

    base_url = args.base_url or str(config.get("axllm_base_url", "http://127.0.0.1:8081"))
    model = args.model or str(config.get("axllm_model", "AXERA-TECH/Qwen3-1.7B"))
    system_prompt = str(config.get("system_prompt", "")).strip()

    if args.system_prompt_file:
        system_prompt = Path(args.system_prompt_file).read_text(encoding="utf-8").strip()

    if not system_prompt:
        print(
            "error: no system prompt available; provide --system-prompt-file or config.system_prompt",
            file=sys.stderr,
        )
        return 2

    selected_ids = {item.strip() for item in args.only.split(",") if item.strip()}
    cases = [case for case in TEST_CASES if not selected_ids or case.case_id in selected_ids]
    if not cases:
        print("error: no test cases selected", file=sys.stderr)
        return 2

    server_ok, server_detail = probe_server(base_url, args.timeout)
    if not server_ok:
        print(f"error: axllm server preflight failed at {base_url}: {server_detail}", file=sys.stderr)
        return 2

    model_ok, model_detail = probe_model(base_url, model, args.timeout)
    if not model_ok:
        print(f"error: model preflight failed: {model_detail}", file=sys.stderr)
        return 2

    reset_result = "not_requested"
    if args.reset_before_run:
        reset_ok, reset_detail = reset_server(base_url, args.timeout)
        reset_result = "ok" if reset_ok else f"failed ({reset_detail})"
        if not reset_ok:
            print(
                f"warning: pre-run reset requested but failed/unsupported: {reset_detail}",
                file=sys.stderr,
            )

    results: list[dict[str, Any]] = []
    shared_messages: list[dict[str, str]] = [{"role": "system", "content": system_prompt}]
    for case in cases:
        user_content = with_json_contract_excerpt(case.user_content)
        if args.disable_thinking:
            user_content = with_no_think_excerpt(user_content)
        if args.cache_mode == "append":
            # Hint the model to treat each case independently while preserving append-only history.
            user_content = (
                f"[Evaluation case: {case.case_id}] "
                "Treat this as a new independent test case and answer only this request.\n"
                f"{user_content}"
            )
            messages = list(shared_messages)
            messages.append({"role": "user", "content": user_content})
        else:
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ]

        try:
            result, assistant_raw = evaluate_case(
                case=case,
                base_url=base_url,
                model=model,
                messages=messages,
                timeout_sec=args.timeout,
                max_tokens=args.max_tokens,
                dump_raw=args.dump_raw,
            )
            if args.cache_mode == "append":
                shared_messages.append({"role": "user", "content": user_content})
                shared_messages.append({"role": "assistant", "content": assistant_raw})
        except Exception as error:
            result = {
                "case_id": case.case_id,
                "description": case.description,
                "elapsed_sec": 0.0,
                "strict_json_ok": False,
                "strict_json_error": str(error),
                "strict_contract_ok": False,
                "strict_contract_error": "Case execution failed before contract validation",
                "parser_ok": False,
                "expect_speech": case.expect_speech,
                "actual_speech": False,
                "speech_ok": False,
                "marker_ok": False,
                "phrases": [],
                "passed": False,
            }
        results.append(result)

    if args.json:
        print(
            json.dumps(
                {
                    "base_url": base_url,
                    "model": model,
                    "cache_mode": args.cache_mode,
                    "max_tokens": max(1, int(args.max_tokens)),
                    "disable_thinking": bool(args.disable_thinking),
                    "reset_before_run": bool(args.reset_before_run),
                    "reset_result": reset_result,
                    "system_prompt_included": True,
                    "results": results,
                },
                indent=2,
            )
        )
    else:
        print(f"Base URL: {base_url}")
        print(f"Model:    {model}")
        print(f"Cache mode: {args.cache_mode}")
        print(f"Max tokens: {max(1, int(args.max_tokens))}")
        print(f"Disable thinking: {bool(args.disable_thinking)}")
        print(f"Reset before run: {bool(args.reset_before_run)} ({reset_result})")
        print("System prompt included: True")
        print_report(results)
        if args.dump_raw:
            print(json.dumps(results, indent=2))

    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())