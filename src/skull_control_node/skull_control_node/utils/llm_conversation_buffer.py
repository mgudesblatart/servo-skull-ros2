import json
import math
import re
from copy import deepcopy


class ConversationBuffer:
    """Sliding conversation window with deterministic structured semantic summary."""

    _WHITESPACE_RE = re.compile(r"\s+")
    _TOPIC_PREFIX_PATTERNS = (
        re.compile(r"^(?:tell me about|explain|describe|summarize|what about)\s+", re.IGNORECASE),
        re.compile(r"^(?:how does|how do|what is|what are)\s+", re.IGNORECASE),
        re.compile(r"^(?:can you explain|can you describe|can you help with)\s+", re.IGNORECASE),
    )
    _QUESTION_STARTERS = (
        "what",
        "why",
        "how",
        "when",
        "where",
        "who",
        "which",
        "can you",
        "could you",
        "would you",
        "tell me",
        "explain",
        "describe",
        "summarize",
    )
    _STOPWORD_TOPICS = {
        "about", "actually", "after", "again", "agent", "briefly", "current", "detail", "details",
        "explain", "hello", "json", "keys", "needed", "only", "phrase", "please", "reply",
        "respond", "servo", "short", "skull", "speak", "speaking", "summary", "system", "there",
        "think", "thoughts", "tool", "tool_calls", "use", "with", "your",
    }
    _FIELD_LIMITS = {
        "user_preferences": 4,
        "active_topics": 6,
        "open_loops": 5,
        "assistant_commitments": 4,
        "known_facts": 6,
    }

    def __init__(
        self,
        *,
        system_prompt: str = "",
        max_turns: int = 4,
        max_window_tokens: int = 800,
        reset_threshold_tokens: int = 150,
        summary_max_chars: int = 600,
    ) -> None:
        self.system_prompt = (system_prompt or "").strip()
        self.max_turns = max(1, int(max_turns))
        self.max_window_tokens = max(128, int(max_window_tokens))
        self.reset_threshold_tokens = max(1, int(reset_threshold_tokens))
        self.summary_max_chars = max(120, int(summary_max_chars))
        self._turns: list[tuple[str, str]] = []
        self._summary_state = self._empty_summary_state()

    def get_history_for_prompt(self) -> list[dict]:
        messages: list[dict] = []
        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})
        summary_text = self._render_summary(self._summary_state)
        if summary_text:
            messages.append(
                {
                    "role": "system",
                    "content": summary_text,
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
        self._summary_state = self._empty_summary_state()

    def reset_with_summary(self, summary) -> None:
        self._turns.clear()
        if isinstance(summary, dict):
            self._summary_state = self._normalize_summary_state(summary)
        else:
            text = self._normalize_text(str(summary or ""))
            state = self._empty_summary_state()
            if text:
                state["known_facts"].append(text)
            self._summary_state = state

        self._shrink_summary_state_to_fit()

    def get_remaining_budget(self, pending_user_content: str = "") -> int:
        messages = self.get_history_for_prompt()
        if pending_user_content:
            messages.append({"role": "user", "content": pending_user_content})
        used_tokens = self._estimate_message_tokens(messages)
        return max(0, self.max_window_tokens - used_tokens)

    def should_reset(self, pending_user_content: str = "") -> bool:
        return self.get_remaining_budget(pending_user_content) < self.reset_threshold_tokens

    def generate_reset_summary(self) -> str:
        return self._render_summary(self.snapshot_summary_state())

    def snapshot_summary_state(self) -> dict:
        state = deepcopy(self._summary_state)
        for user_content, assistant_content in self._turns:
            self._merge_turn_into_state(state, user_content, assistant_content)
        return self._normalize_summary_state(state)

    @property
    def summary_text(self) -> str:
        return self._render_summary(self._summary_state)

    @property
    def summary_state(self) -> dict:
        return deepcopy(self._summary_state)

    @property
    def recent_turns(self) -> list[tuple[str, str]]:
        return list(self._turns)

    def _trim_window(self) -> None:
        evicted: list[tuple[str, str]] = []
        while self._turns and (
            len(self._turns) > self.max_turns
            or self._estimate_message_tokens(self.get_history_for_prompt()) > self.max_window_tokens
        ):
            evicted.append(self._turns.pop(0))

        if evicted:
            for user_content, assistant_content in evicted:
                self._merge_turn_into_state(self._summary_state, user_content, assistant_content)

        self._shrink_summary_state_to_fit()

    @classmethod
    def _empty_summary_state(cls) -> dict:
        return {
            "user_preferences": [],
            "active_topics": [],
            "open_loops": [],
            "assistant_commitments": [],
            "known_facts": [],
        }

    def _normalize_summary_state(self, summary_state: dict) -> dict:
        normalized = self._empty_summary_state()
        if not isinstance(summary_state, dict):
            return normalized

        for field, limit in self._FIELD_LIMITS.items():
            values = summary_state.get(field, [])
            if not isinstance(values, list):
                continue
            for value in values:
                self._append_unique(normalized[field], self._normalize_text(str(value or "")), limit)
        return normalized

    def _shrink_summary_state_to_fit(self) -> None:
        field_priority = [
            "known_facts",
            "active_topics",
            "assistant_commitments",
            "open_loops",
            "user_preferences",
        ]
        while self._render_summary(self._summary_state) and self._estimate_message_tokens(self.get_history_for_prompt()) > self.max_window_tokens:
            trimmed = False
            for field in field_priority:
                if self._summary_state[field]:
                    self._summary_state[field].pop(0)
                    trimmed = True
                    break
            if not trimmed:
                break

    @classmethod
    def _append_unique(cls, items: list[str], value: str, limit: int) -> None:
        value = cls._normalize_text(value)
        if not value:
            return
        lowered = value.casefold()
        for existing in items:
            if existing.casefold() == lowered:
                return
        items.append(value)
        if len(items) > limit:
            del items[0 : len(items) - limit]

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

    def _extract_user_preferences(self, user_text: str) -> list[str]:
        preferences: list[str] = []
        lower_text = user_text.lower()
        if "keep it short" in lower_text or "be concise" in lower_text or "briefly" in lower_text:
            preferences.append("prefers concise responses")

        match = re.search(r"\bprefer(?:s|red)?\s+([^.!?]+)", user_text, re.IGNORECASE)
        if match:
            preferences.append(f"prefers {self._compact_text(match.group(1), 60)}")

        for match in re.finditer(r"\b(?:do not|don't|avoid)\s+([^.!?]+)", user_text, re.IGNORECASE):
            preferences.append(f"avoid {self._compact_text(match.group(1), 60)}")

        return preferences

    def _extract_topic(self, user_text: str) -> str:
        text = self._normalize_text(user_text)
        if not text:
            return ""

        candidate = text.rstrip("?.!")
        for pattern in self._TOPIC_PREFIX_PATTERNS:
            candidate = pattern.sub("", candidate)

        candidate = self._normalize_text(candidate)
        if not candidate:
            return ""

        words = [
            word for word in re.findall(r"[A-Za-z0-9']+", candidate.lower())
            if len(word) >= 4 and word not in self._STOPWORD_TOPICS
        ]
        if not words:
            return self._compact_text(candidate, 50)
        return " ".join(words[:4])

    def _extract_open_loop(self, user_text: str) -> str:
        text = self._normalize_text(user_text)
        if not text:
            return ""
        lower_text = text.lower()
        if text.endswith("?") or lower_text.startswith(self._QUESTION_STARTERS):
            return self._compact_text(text, 90)
        return ""

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

    def _extract_assistant_commitment(self, assistant_summary: str) -> str:
        summary = self._normalize_text(assistant_summary)
        if not summary:
            return ""
        lowered = summary.lower()
        if lowered.startswith(("i can", "i will", "i'll", "we can", "let me", "let's")):
            return self._compact_text(summary, 90)
        return ""

    def _extract_known_fact(self, assistant_summary: str) -> str:
        summary = self._normalize_text(assistant_summary)
        if not summary:
            return ""
        return self._compact_text(summary, 90)

    def _merge_turn_into_state(self, summary_state: dict, user_content: str, assistant_content: str) -> None:
        user_text = self._compact_text(user_content, 120)
        assistant_summary = self._assistant_summary_text(assistant_content)

        for preference in self._extract_user_preferences(user_text):
            self._append_unique(summary_state["user_preferences"], preference, self._FIELD_LIMITS["user_preferences"])

        topic = self._extract_topic(user_text)
        if topic:
            self._append_unique(summary_state["active_topics"], topic, self._FIELD_LIMITS["active_topics"])

        open_loop = self._extract_open_loop(user_text)
        if open_loop:
            self._append_unique(summary_state["open_loops"], open_loop, self._FIELD_LIMITS["open_loops"])

        commitment = self._extract_assistant_commitment(assistant_summary)
        if commitment:
            self._append_unique(
                summary_state["assistant_commitments"],
                commitment,
                self._FIELD_LIMITS["assistant_commitments"],
            )

        known_fact = self._extract_known_fact(assistant_summary)
        if known_fact:
            self._append_unique(summary_state["known_facts"], known_fact, self._FIELD_LIMITS["known_facts"])

    def _render_summary(self, summary_state: dict) -> str:
        normalized = self._normalize_summary_state(summary_state)
        lines: list[str] = []
        field_labels = {
            "user_preferences": "User preferences",
            "active_topics": "Active topics",
            "open_loops": "Open loops",
            "assistant_commitments": "Assistant commitments",
            "known_facts": "Known facts",
        }
        for field in (
            "user_preferences",
            "active_topics",
            "open_loops",
            "assistant_commitments",
            "known_facts",
        ):
            values = normalized[field]
            if values:
                lines.append(f"- {field_labels[field]}: {'; '.join(values)}")
        if not lines:
            return ""
        rendered = "Conversation summary:\n" + "\n".join(lines)
        if len(rendered) <= self.summary_max_chars:
            return rendered
        return rendered[: self.summary_max_chars - 3].rstrip() + "..."

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