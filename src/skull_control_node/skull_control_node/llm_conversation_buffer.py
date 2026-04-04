import json
import math
import re


class ConversationBuffer:
    """Sliding conversation window with lightweight summarization of evicted turns."""

    _WHITESPACE_RE = re.compile(r"\s+")

    def __init__(
        self,
        *,
        system_prompt: str = "",
        max_turns: int = 4,
        max_window_tokens: int = 1500,
        reset_threshold_tokens: int = 200,
        summary_max_chars: int = 600,
    ) -> None:
        self.system_prompt = (system_prompt or "").strip()
        self.max_turns = max(1, int(max_turns))
        self.max_window_tokens = max(128, int(max_window_tokens))
        self.reset_threshold_tokens = max(1, int(reset_threshold_tokens))
        self.summary_max_chars = max(120, int(summary_max_chars))
        self._turns: list[tuple[str, str]] = []
        self._summary = ""

    def get_history_for_prompt(self) -> list[dict]:
        messages: list[dict] = []
        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})
        if self._summary:
            messages.append(
                {
                    "role": "system",
                    "content": f"Conversation summary: {self._summary}",
                }
            )
        for user_content, assistant_content in self._turns:
            messages.append({"role": "user", "content": user_content})
            messages.append({"role": "assistant", "content": assistant_content})
        return messages

    def add_turn(self, user_content: str, assistant_content: str) -> None:
        self._turns.append((user_content, assistant_content))
        self._trim_window()

    def clear(self) -> None:
        self._turns.clear()
        self._summary = ""

    def reset_with_summary(self, summary: str) -> None:
        self._turns.clear()
        normalized = self._normalize_text(summary)
        if len(normalized) > self.summary_max_chars:
            normalized = normalized[-self.summary_max_chars :].lstrip()
        self._summary = normalized

        while self._summary and self._estimate_message_tokens(self.get_history_for_prompt()) > self.max_window_tokens:
            self._summary = self._shrink_summary(self._summary)

    def get_remaining_budget(self, pending_user_content: str = "") -> int:
        messages = self.get_history_for_prompt()
        if pending_user_content:
            messages.append({"role": "user", "content": pending_user_content})
        used_tokens = self._estimate_message_tokens(messages)
        return max(0, self.max_window_tokens - used_tokens)

    def should_reset(self, pending_user_content: str = "") -> bool:
        return self.get_remaining_budget(pending_user_content) < self.reset_threshold_tokens

    def generate_reset_summary(self) -> str:
        parts: list[str] = []
        if self._summary:
            parts.append(self._summary)
        for user_content, assistant_content in self._turns:
            parts.append(self._summarize_turn(user_content, assistant_content))
        combined = " ".join(part for part in parts if part).strip()
        if len(combined) <= self.summary_max_chars:
            return combined
        return combined[-self.summary_max_chars :].lstrip()

    @property
    def summary_text(self) -> str:
        return self._summary

    def _trim_window(self) -> None:
        evicted: list[tuple[str, str]] = []
        while self._turns and (
            len(self._turns) > self.max_turns
            or self._estimate_message_tokens(self.get_history_for_prompt()) > self.max_window_tokens
        ):
            evicted.append(self._turns.pop(0))

        if evicted:
            self._summary = self._merge_summary(self._summary, evicted)

        while self._summary and self._estimate_message_tokens(self.get_history_for_prompt()) > self.max_window_tokens:
            self._summary = self._shrink_summary(self._summary)

    def _merge_summary(self, existing_summary: str, evicted_turns: list[tuple[str, str]]) -> str:
        parts: list[str] = []
        if existing_summary:
            parts.append(existing_summary)
        parts.extend(self._summarize_turn(user_content, assistant_content) for user_content, assistant_content in evicted_turns)
        combined = " ".join(part for part in parts if part).strip()
        if len(combined) <= self.summary_max_chars:
            return combined
        return combined[-self.summary_max_chars :].lstrip()

    def _shrink_summary(self, summary: str) -> str:
        if len(summary) <= 80:
            return ""
        shortened = summary[len(summary) // 4 :].lstrip()
        if len(shortened) <= self.summary_max_chars:
            return shortened
        return shortened[-self.summary_max_chars :].lstrip()

    @classmethod
    def _normalize_text(cls, text: str) -> str:
        normalized = cls._WHITESPACE_RE.sub(" ", (text or "").strip())
        return normalized.strip()

    @classmethod
    def _compact_text(cls, text: str, max_chars: int = 120) -> str:
        normalized = cls._normalize_text(text.split("\n\n", 1)[0])
        if len(normalized) <= max_chars:
            return normalized
        return normalized[: max_chars - 3].rstrip() + "..."

    @classmethod
    def _assistant_summary_text(cls, assistant_content: str) -> str:
        try:
            parsed = json.loads(assistant_content)
        except json.JSONDecodeError:
            return cls._compact_text(assistant_content, 90)

        if isinstance(parsed, dict):
            tool_calls = parsed.get("tool_calls")
            if isinstance(tool_calls, list):
                for call in tool_calls:
                    if not isinstance(call, dict):
                        continue
                    say_phrase = call.get("say_phrase")
                    if isinstance(say_phrase, dict):
                        msg = say_phrase.get("msg")
                        if isinstance(msg, str) and msg.strip():
                            return cls._compact_text(msg, 90)

            thoughts = parsed.get("thoughts")
            if isinstance(thoughts, str) and thoughts.strip():
                return cls._compact_text(thoughts, 90)

        return cls._compact_text(assistant_content, 90)

    def _summarize_turn(self, user_content: str, assistant_content: str) -> str:
        user_summary = self._compact_text(user_content, 80)
        assistant_summary = self._assistant_summary_text(assistant_content)
        if not user_summary and not assistant_summary:
            return ""
        if not assistant_summary:
            return f"User asked: {user_summary}."
        if not user_summary:
            return f"Assistant replied: {assistant_summary}."
        return f"User asked: {user_summary}. Assistant replied: {assistant_summary}."

    @staticmethod
    def _estimate_text_tokens(text: str) -> int:
        normalized = (text or "").strip()
        if not normalized:
            return 0
        return max(1, int(math.ceil(len(normalized) / 4.0)))

    def _estimate_message_tokens(self, messages: list[dict]) -> int:
        total = 0
        for message in messages:
            total += 4
            total += self._estimate_text_tokens(str(message.get("content", "")))
        return total + 2