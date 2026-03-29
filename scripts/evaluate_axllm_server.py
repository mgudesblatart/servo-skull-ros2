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
REQUIRED_JSON_KEYS = {"thoughts", "tool_calls"}


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


def with_json_contract_excerpt(text: str) -> str:
    stripped = text.strip()
    if NODE_JSON_CONTRACT_EXCERPT in stripped:
        return stripped
    return f"{stripped} {NODE_JSON_CONTRACT_EXCERPT}".strip()


def request_chat_completion(
    *,
    base_url: str,
    model: str,
    system_prompt: str,
    user_content: str,
    timeout_sec: float,
) -> tuple[float, str]:
    messages: list[dict[str, str]] = [{"role": "system", "content": system_prompt}]
    messages.append({"role": "user", "content": user_content})

    payload = {
        "model": model,
        "messages": messages,
        "stream": False,
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


def evaluate_case(
    *,
    case: TestCase,
    base_url: str,
    model: str,
    system_prompt: str,
    timeout_sec: float,
    dump_raw: bool,
) -> dict[str, Any]:
    elapsed_sec, raw_content = request_chat_completion(
        base_url=base_url,
        model=model,
        system_prompt=system_prompt,
        user_content=with_json_contract_excerpt(case.user_content),
        timeout_sec=timeout_sec,
    )

    stripped = strip_think_blocks(raw_content)
    strict_json_ok = False
    strict_json_error = ""
    parsed_strict: dict[str, Any] | None = None
    strict_contract_ok = False
    strict_contract_error = ""
    try:
        parsed_candidate = json.loads(stripped)
        if isinstance(parsed_candidate, dict):
            parsed_strict = parsed_candidate
            strict_json_ok = True
        else:
            strict_json_error = f"Top-level JSON was {type(parsed_candidate).__name__}, expected object"
    except json.JSONDecodeError as error:
        strict_json_error = str(error)

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
    return result


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

    results: list[dict[str, Any]] = []
    for case in cases:
        try:
            result = evaluate_case(
                case=case,
                base_url=base_url,
                model=model,
                system_prompt=system_prompt,
                timeout_sec=args.timeout,
                dump_raw=args.dump_raw,
            )
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
                    "system_prompt_included": True,
                    "results": results,
                },
                indent=2,
            )
        )
    else:
        print(f"Base URL: {base_url}")
        print(f"Model:    {model}")
        print("System prompt included: True")
        print_report(results)
        if args.dump_raw:
            print(json.dumps(results, indent=2))

    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())