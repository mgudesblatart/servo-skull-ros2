from typing import Callable

from skull_control_node.utils.response_processing_stage import ProcessedResponseArtifacts


def persist_and_publish_response(
    *,
    artifacts: ProcessedResponseArtifacts,
    user_content: str,
    append_turn_fn: Callable[..., None],
    cancel_requested_event,
    logger,
    publish_tts_phrase_fn: Callable[[str], None],
) -> None:
    if artifacts.compact_assistant_text.strip():
        append_turn_fn(
            prompt_user_content=user_content,
            compact_user_content=artifacts.history_user_content,
            raw_assistant_content=artifacts.raw_assistant_text,
            compact_assistant_content=artifacts.compact_assistant_text,
        )

    for phrase in artifacts.phrases:
        if cancel_requested_event.is_set():
            logger.warning("Cancel received during publish loop; stopping.")
            break
        publish_tts_phrase_fn(phrase)
        logger.info(f"Published TTS phrase: {phrase}")
