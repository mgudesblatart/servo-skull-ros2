from dataclasses import dataclass
from typing import Callable


@dataclass
class GenerationResult:
    ok: bool
    raw_response: str = ""
    reason: str = ""


def run_generation(
    *,
    user_content: str,
    build_messages: Callable[[str], list[dict]],
    http_generate_chat: Callable[[list[dict]], str],
    request_in_flight_event,
    cancel_requested_event,
    logger,
    publish_status: Callable[..., None],
) -> GenerationResult:
    cancelled_after_response = False
    try:
        request_in_flight_event.set()
        messages = build_messages(user_content)
        raw_response = http_generate_chat(messages)
    except Exception as error:
        if cancel_requested_event.is_set():
            logger.warning(f"HTTP request cancelled: {error}")
            publish_status("cancelled", reason="control_cancel", detail=str(error))
            return GenerationResult(ok=False, reason="cancelled")

        logger.error(f"HTTP inference error: {error}")
        publish_status("error", reason="http_inference_error", detail=str(error))
        return GenerationResult(ok=False, reason="error")
    finally:
        request_in_flight_event.clear()
        if cancel_requested_event.is_set():
            cancel_requested_event.clear()
            logger.warning("HTTP response received but cancel was set; dropping.")
            cancelled_after_response = True

    if cancelled_after_response:
        return GenerationResult(ok=False, reason="cancelled_after_response")

    return GenerationResult(ok=True, raw_response=raw_response)
