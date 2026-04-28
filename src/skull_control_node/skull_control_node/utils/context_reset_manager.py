import threading
from typing import Callable

from skull_control_node.utils.context_input_helpers import (
    estimate_message_tokens,
    extract_untruncated_turns_from_append_history,
)
from skull_control_node.utils.formatting_parsing_helpers import compact_detail
from skull_control_node.utils.summary_refinement_helpers import (
    build_summary_refinement_messages,
    format_summary_state_as_text,
    parse_summary_refinement_response,
)


class ContextResetManager:
    """Owns append-history budgeting, reset flow, and summary refinement state."""

    def __init__(
        self,
        *,
        system_prompt: str,
        max_window_tokens: int,
        conversation_buffer,
        http_client,
        history_lock: threading.Lock,
        publish_status: Callable[..., None],
        logger,
        summary_fields: tuple[str, ...],
        summary_refinement_hint: str,
        no_think_excerpt: str,
        default_disable_thinking: bool,
        summary_refinement_disable_thinking: bool,
        summary_refinement_request_timeout_sec: float,
        summary_refinement_max_output_tokens: int,
        summary_refinement_unbounded_when_thinking: bool,
        enable_llm_summary_refinement: bool,
    ) -> None:
        self.system_prompt = system_prompt
        self.max_window_tokens = max_window_tokens
        self.conversation_buffer = conversation_buffer
        self.http_client = http_client
        self.history_lock = history_lock
        self.publish_status = publish_status
        self.logger = logger

        self.summary_fields = summary_fields
        self.summary_refinement_hint = summary_refinement_hint
        self.no_think_excerpt = no_think_excerpt
        self.default_disable_thinking = default_disable_thinking
        self.summary_refinement_disable_thinking = summary_refinement_disable_thinking
        self.summary_refinement_request_timeout_sec = summary_refinement_request_timeout_sec
        self.summary_refinement_max_output_tokens = summary_refinement_max_output_tokens
        self.summary_refinement_unbounded_when_thinking = summary_refinement_unbounded_when_thinking
        self.enable_llm_summary_refinement = enable_llm_summary_refinement

        self.append_history: list[dict] = []
        self.budget_warning_level = "ok"
        self.reset_append_history_root()

    def build_messages(self, user_content: str) -> list[dict]:
        with self.history_lock:
            messages: list[dict] = list(self.append_history)
        messages.append({"role": "user", "content": user_content})
        return messages

    def append_turn(
        self,
        *,
        prompt_user_content: str,
        compact_user_content: str,
        raw_assistant_content: str,
        compact_assistant_content: str,
    ) -> None:
        with self.history_lock:
            self.append_history.append({"role": "user", "content": prompt_user_content})
            self.append_history.append({"role": "assistant", "content": raw_assistant_content})
            self.conversation_buffer.add_turn(compact_user_content, compact_assistant_content)

        remaining_budget = self.get_append_remaining_budget()
        self.publish_budget_status(remaining_budget)

    def clear_history(self, reason: str = "") -> None:
        with self.history_lock:
            self.conversation_buffer.clear()
            self.reset_append_history_root()
        self.budget_warning_level = "ok"
        self.logger.info(f"Conversation history cleared. reason={reason or 'unknown'}")

    def reset_append_history_root(self, summary_text: str = "") -> None:
        root: list[dict] = []
        if self.system_prompt:
            root.append({"role": "system", "content": self.system_prompt})
        if summary_text:
            root.append({"role": "system", "content": summary_text})
        self.append_history = root

    def get_append_remaining_budget(self, pending_user_content: str = "") -> int:
        with self.history_lock:
            messages = list(self.append_history)
        if pending_user_content:
            messages.append({"role": "user", "content": pending_user_content})
        used_tokens = estimate_message_tokens(messages)
        return max(0, self.max_window_tokens - used_tokens)

    def publish_budget_status(self, remaining_budget: int) -> None:
        low_threshold = max(200, self.max_window_tokens // 4)
        next_level = "ok"
        if remaining_budget < self.conversation_buffer.reset_threshold_tokens:
            next_level = "critical"
        elif remaining_budget < low_threshold:
            next_level = "low"

        if next_level == self.budget_warning_level:
            return

        self.budget_warning_level = next_level
        if next_level == "low":
            detail = f"remaining_tokens={remaining_budget};threshold={low_threshold}"
            self.publish_status("warning", reason="context_budget_low", detail=detail)
            self.logger.info(
                f"Conversation buffer budget getting tight: remaining_tokens={remaining_budget}"
            )
        elif next_level == "critical":
            detail = (
                f"remaining_tokens={remaining_budget};"
                f"threshold={self.conversation_buffer.reset_threshold_tokens}"
            )
            self.publish_status("warning", reason="context_budget_critical", detail=detail)
            self.logger.warning(
                f"Conversation buffer critically low before reset: remaining_tokens={remaining_budget}"
            )

    def perform_context_reset(self, pending_user_content: str) -> None:
        with self.history_lock:
            summary_state = self.conversation_buffer.snapshot_summary_state()
            prior_summary_state = self.conversation_buffer.summary_state
            untruncated_turns = extract_untruncated_turns_from_append_history(self.append_history)
            before_messages = list(self.append_history)
            if pending_user_content:
                before_messages.append({"role": "user", "content": pending_user_content})
            remaining_before = max(0, self.max_window_tokens - estimate_message_tokens(before_messages))

            refined_summary_state = self.refine_summary_state_with_llm(
                prior_summary_state,
                untruncated_turns,
                summary_state,
            )
            final_summary_state = refined_summary_state or summary_state
            self.conversation_buffer.reset_with_summary(final_summary_state)

            summary_text = format_summary_state_as_text(final_summary_state)
            self.reset_append_history_root(summary_text)
            after_messages = list(self.append_history)
            if pending_user_content:
                after_messages.append({"role": "user", "content": pending_user_content})
            remaining_after = max(0, self.max_window_tokens - estimate_message_tokens(after_messages))

        self.publish_status(
            "warning",
            reason="context_reset_requested",
            detail=f"remaining_before={remaining_before}",
        )
        backend_reset_ok = self.http_client.reset()
        self.budget_warning_level = "ok"
        self.publish_status(
            "reset",
            reason="context_budget_reset",
            detail=(
                f"remaining_before={remaining_before};"
                f"remaining_after={remaining_after};"
                f"summary_seeded={'yes' if bool(summary_text) else 'no'};"
                f"summary_refined={'yes' if refined_summary_state is not None else 'no'};"
                f"backend_reset_ok={backend_reset_ok}"
            ),
        )
        self.logger.warning(
            "Context budget exhausted; reset conversation history with summary seed. "
            f"remaining_before={remaining_before} remaining_after={remaining_after} "
            f"backend_reset_ok={backend_reset_ok}"
        )
        self.publish_budget_status(remaining_after)

    def refine_summary_state_with_llm(
        self,
        prior_summary_state: dict,
        untruncated_turns: list[tuple[str, str]],
        heuristic_summary_state: dict,
    ) -> dict | None:
        if not self.enable_llm_summary_refinement:
            return None
        if not any(heuristic_summary_state.get(field) for field in self.summary_fields):
            return None

        try:
            messages = build_summary_refinement_messages(
                prior_summary_state=prior_summary_state,
                untruncated_turns=untruncated_turns,
                heuristic_summary_state=heuristic_summary_state,
                summary_refinement_hint=self.summary_refinement_hint,
                default_disable_thinking=self.default_disable_thinking,
                no_think_excerpt=self.no_think_excerpt,
                disable_thinking=self.summary_refinement_disable_thinking,
            )

            refinement_max_tokens = self.summary_refinement_max_output_tokens
            if self.summary_refinement_unbounded_when_thinking and not self.summary_refinement_disable_thinking:
                refinement_max_tokens = 0

            raw_response = self.http_client.generate_chat(
                messages,
                max_output_tokens=refinement_max_tokens,
                timeout_sec=self.summary_refinement_request_timeout_sec,
            )
        except Exception as error:
            self.publish_status(
                "warning",
                reason="context_summary_refine_failed",
                detail=str(error),
            )
            self.logger.warning(f"LLM summary refinement failed before reset: {error}")
            return None

        refined = parse_summary_refinement_response(
            raw_response,
            normalize_summary_state_fn=self.conversation_buffer._normalize_summary_state,
            summary_fields=self.summary_fields,
        )
        if refined is None:
            self.publish_status(
                "warning",
                reason="context_summary_refine_invalid",
                detail=compact_detail(raw_response),
            )
            self.logger.warning(
                "LLM summary refinement returned invalid JSON; using heuristic summary."
            )
            return None

        return refined
