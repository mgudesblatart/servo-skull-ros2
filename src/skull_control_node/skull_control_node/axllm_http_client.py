"""
axllm_http_client.py

HTTP client wrapper for axllm server mode.

Primary target:
  - OpenAI-compatible endpoints exposed by `axllm serve`:
    - GET  /v1/models
    - POST /v1/chat/completions

Best-effort reset support:
  - POST /reset
  - POST /api/reset
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request


class AxllmHttpClient:
    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        startup_timeout_sec: float = 30.0,
        request_timeout_sec: float = 60.0,
        max_output_tokens: int = 128,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model.strip() or "default"
        self.startup_timeout_sec = startup_timeout_sec
        self.request_timeout_sec = request_timeout_sec
        self.max_output_tokens = max(1, int(max_output_tokens))

    def _request_json(
        self,
        method: str,
        path: str,
        payload: dict | None = None,
        timeout_sec: float | None = None,
    ) -> tuple[int, dict | list | str | None]:
        url = f"{self.base_url}{path}"
        data_bytes = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data_bytes = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"

        req = urllib.request.Request(
            url=url, method=method, data=data_bytes, headers=headers
        )
        timeout = timeout_sec if timeout_sec is not None else self.request_timeout_sec

        try:
            with urllib.request.urlopen(req, timeout=timeout) as response:
                status = getattr(response, "status", 200)
                raw = response.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as error:
            body = (
                error.read().decode("utf-8", errors="replace")
                if hasattr(error, "read")
                else ""
            )
            try:
                return error.code, json.loads(body)
            except Exception:
                return error.code, body

        if not raw.strip():
            return status, None

        try:
            return status, json.loads(raw)
        except Exception:
            return status, raw

    def start(self) -> None:
        """Wait until axllm serve reports model endpoint readiness."""
        deadline = time.monotonic() + self.startup_timeout_sec
        last_error = "Unknown error"

        while time.monotonic() < deadline:
            try:
                status, payload = self._request_json(
                    "GET", "/v1/models", timeout_sec=2.0
                )
                if status == 200 and isinstance(payload, dict):
                    return
                last_error = (
                    f"Unexpected readiness response: status={status}, payload={payload}"
                )
            except Exception as error:
                last_error = str(error)
            time.sleep(0.2)

        raise TimeoutError(f"Timed out waiting for axllm HTTP readiness: {last_error}")

    def generate_chat(
        self,
        messages: list[dict],
        max_output_tokens: int | None = None,
        timeout_sec: float | None = None,
    ) -> str:
        """
        Send a full OpenAI-style messages list and return the assistant reply text.

        Args:
            messages: List of {"role": ..., "content": ...} dicts.
                      Roles: "system", "user", "assistant".
            max_output_tokens: Token budget control (None=default, >0=cap, <=0=unbounded).
            timeout_sec: Optional per-call timeout in seconds (overrides client default).
        """
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
        }

        # Token budget controls:
        # - max_output_tokens is None: use client default cap
        # - max_output_tokens > 0: use explicit cap
        # - max_output_tokens <= 0: omit max_tokens field (server-side default/unbounded)
        if max_output_tokens is None:
            payload["max_tokens"] = self.max_output_tokens
        elif int(max_output_tokens) > 0:
            payload["max_tokens"] = max(1, int(max_output_tokens))

        status, response = self._request_json(
            "POST", "/v1/chat/completions", payload=payload, timeout_sec=timeout_sec
        )
        if status != 200:
            raise RuntimeError(
                f"axllm chat completion failed: status={status}, body={response}"
            )

        if not isinstance(response, dict):
            raise RuntimeError(
                f"Invalid axllm response type: {type(response).__name__}"
            )

        choices = response.get("choices")
        if not isinstance(choices, list) or not choices:
            raise RuntimeError(f"axllm response missing choices: {response}")

        message = choices[0].get("message") if isinstance(choices[0], dict) else None
        content = message.get("content") if isinstance(message, dict) else None
        if not isinstance(content, str):
            raise RuntimeError(f"axllm response missing message content: {response}")

        return content

    def generate(self, prompt: str) -> str:
        """Single-turn convenience wrapper. Prefer generate_chat() for multi-turn use."""
        payload = {
            "model": self.model,
            "messages": [
                {"role": "user", "content": prompt},
            ],
            "stream": False,
            "max_tokens": self.max_output_tokens,
        }

        status, response = self._request_json(
            "POST", "/v1/chat/completions", payload=payload
        )
        if status != 200:
            raise RuntimeError(
                f"axllm chat completion failed: status={status}, body={response}"
            )

        if not isinstance(response, dict):
            raise RuntimeError(
                f"Invalid axllm response type: {type(response).__name__}"
            )

        choices = response.get("choices")
        if not isinstance(choices, list) or not choices:
            raise RuntimeError(f"axllm response missing choices: {response}")

        message = choices[0].get("message") if isinstance(choices[0], dict) else None
        content = message.get("content") if isinstance(message, dict) else None
        if not isinstance(content, str):
            raise RuntimeError(f"axllm response missing message content: {response}")

        return content

    def reset(self) -> bool:
        """
        Best-effort reset hook.
        OpenAI-compatible axllm may not expose reset; vendor API often does.
        """
        for path in ("/reset", "/api/reset"):
            try:
                status, _ = self._request_json("POST", path, payload={})
                if status == 200:
                    return True
            except Exception:
                continue
        return False

    def close(self) -> None:
        """No persistent socket/process to close in simple urllib mode."""
        return
