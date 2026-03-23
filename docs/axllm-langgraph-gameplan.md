# AXLLM + LangGraph Integration Gameplan

Date: 2026-03-22

## Purpose

Define a practical path for moving from the current AXCL subprocess runtime to a maintainable stack with:

1. Reliable startup scripts for tokenizer + LLM server.
2. Optional protocol mapper if `axllm serve` binary remains unavailable.
3. A separate LangChain/LangGraph orchestration layer.
4. A ROS2 `llm_agent_axcl_node` integration layer that talks to the orchestration service.

## Current Known Constraints

- Vendor model bundle server works on `:8000` but uses AXERA endpoints (for example `/api/chat`, `/api/reset`, `/api/stop`).
- OpenAI-style endpoints (`/v1/chat/completions`) are not available in that vendor API binary.
- Existing ROS node currently manages runtime directly through subprocess stdin/stdout.
- Goal is to keep LangChain/LangGraph separate from ROS node internals.

## Architecture Target

- Runtime layer: AXERA server (`axllm serve` if available, otherwise vendor `main_api_axcl_aarch64`).
- Compatibility layer: Optional OpenAI proxy/mapper (only if needed).
- Orchestration layer: LangGraph service (memory, tool routing, context policy).
- ROS layer: thin client node that forwards envelopes and emits speech/tool events.

---

## Workstream 1: Startup Script (Tokenizer + Server)

### Deliverable

Create `scripts/start_axllm_stack.sh` with:

- Startup mode switch:
  - `backend=openai` (upstream `axllm serve` path)
  - `backend=vendor_api` (vendor `main_api_axcl_aarch64` path)
- Tokenizer launch and readiness check.
- LLM server launch and readiness check.
- Optional log and pid-file management.
- Clean shutdown trap.

### Script behavior

1. Parse env vars or flags:
   - `MODEL_DIR`
   - `BACKEND_MODE`
   - `AXLLM_PORT` (default `8000`)
   - `TOKENIZER_PORT` (default `12345`)
   - `DEVICE_IDS` (default `0`)
   - `USE_INT4` (default `1`)
2. Start tokenizer (`qwen2.5_tokenizer_uid.py`) with venv python.
3. Wait for tokenizer readiness (`tcp 12345` connect test).
4. Start selected server:
   - OpenAI mode: `axllm serve <model_dir> --port <AXLLM_PORT>`
   - Vendor mode: `main_api_axcl_aarch64 --template_filename_axmodel ...`
5. Wait for server readiness:
   - OpenAI mode: expect success on `/v1/models`.
   - Vendor mode: expect non-connection failure and known route behavior (for example `/api/reset` 200 on POST).
6. Print summary (ports, mode, pids, log paths).

### Acceptance criteria

- One command starts tokenizer + server reliably.
- Re-running script cleans stale pids and restarts cleanly.
- Script exits non-zero with actionable error text when any step fails.

---

## Workstream 2: Optional Proxy/Mapper

### When to build

Build only if either is true:

- You cannot get a working upstream `axllm serve` binary.
- You need OpenAI-compatible API for LangChain integration now.

### Deliverable

Create `services/axllm_openai_proxy.py` (FastAPI):

- Implements:
  - `GET /v1/models`
  - `POST /v1/chat/completions`
- Maps requests to vendor API:
  - Reset path: `POST /api/reset` with `system_prompt` when needed.
  - Chat path: `POST /api/chat` with `messages`.
  - Optional stop path: `GET /api/stop` for cancellation.

### Mapping strategy

1. Translate OpenAI messages to vendor `messages` format.
2. Preserve stateless behavior by default (send complete message list each call).
3. Optional session mode:
   - Keep a short-lived in-memory session map keyed by `conversation_id`.
4. Normalize response back into OpenAI `chat.completion` schema.

### Risks and guardrails

- Vendor API may have hidden limits or non-standard behavior.
- Keep proxy minimal: no business logic, no memory policy.
- Add explicit timeouts and retries with bounded backoff.

### Acceptance criteria

- `POST /v1/chat/completions` returns stable OpenAI-shaped JSON.
- LangChain `ChatOpenAI` can call the proxy with `base_url` override.

---

## Workstream 3: LangChain/LangGraph Service Layer

### Deliverable

Create separate orchestration service (non-ROS package), suggested path:

- `services/langgraph_agent/`
  - `app.py` (HTTP service)
  - `graph.py` (LangGraph definition)
  - `state.py` (typed state)
  - `memory.py` (summary + person memory)
  - `tools.py` (tool contracts)
  - `backend_client.py` (OpenAI or proxy client)

### Responsibilities

1. Conversation state policy:
   - Recent turn window.
   - Summarization threshold and rollup.
2. Long-term memory:
   - Person facts keyed by `person_id` (and later `name` once recognition exists).
3. Tool orchestration:
   - Structured tool calls, validation, retries.
4. Output contract:
   - Stable response schema for ROS node (speech actions + metadata).

### Initial graph nodes

1. `ingest_input`
2. `fetch_context`
3. `trim_or_summarize`
4. `call_model`
5. `validate_actions`
6. `execute_tools` (or mark for ROS execution)
7. `persist_memory`
8. `emit_response`

### Storage plan

- Phase 1: local SQLite for memory + checkpoints.
- Phase 2: optional Redis/Postgres if multi-process persistence is needed.

### Acceptance criteria

- Handles multi-turn context without runaway token growth.
- Produces deterministic structured responses for ROS consumption.
- Memory updates survive process restart.

---

## Workstream 4: ROS2 Node Integration Layer

### Deliverable

Refactor `llm_agent_axcl_node` behavior into a thin transport client:

- Keep ROS pub/sub and cancellation semantics.
- Remove direct model prompting/parsing logic from node.
- Forward envelopes to LangGraph service and apply returned actions.

### Node responsibilities after refactor

1. Subscribe to `/skull_control/llm_input` and `/llm_agent_axcl/control`.
2. Convert incoming envelope into orchestration request DTO.
3. Send request to LangGraph service (`/invoke` endpoint).
4. Publish resulting speech content to `/text_to_speech/text_input`.
5. Preserve interrupt behavior:
   - Cancel in-flight requests.
   - Propagate stop to backend/proxy if supported.

### API contract between ROS and orchestration

Request:

- `event_id`
- `channel`
- `urgency`
- `text`
- `tracked_people`
- `fsm_snapshot`

Response:

- `final_output`
- `speech_actions[]`
- `tool_actions[]`
- `debug` (optional)

### Acceptance criteria

- Existing FSM and interrupt semantics remain intact.
- Node can switch backend by config only (no code change).
- No direct AXCL subprocess management left in ROS node path.

---

## Implementation Order (Recommended)

1. Build Workstream 1 script and lock startup reliability.
2. Decide backend branch:
   - If upstream `axllm serve` works: skip proxy.
   - If not: implement Workstream 2 proxy.
3. Build Workstream 3 LangGraph service against chosen API shape.
4. Integrate Workstream 4 ROS thin client.
5. Run end-to-end speech + interrupt + context retention tests.

## Testing Plan

### Smoke

- Start stack script.
- Verify tokenizer and server readiness.
- Send one chat request and validate response latency.

### Integration

- ROS input envelope -> orchestration service -> TTS output.
- Interrupt during generation -> stop/cancel observed across layers.

### Regression

- Repeated system events do not cause repetitive speech.
- Long conversation triggers summarization, not context overflow.
- Restart service and verify persisted memory continuity.

## Rollback Strategy

- Keep current direct subprocess `llm_agent_axcl_node` path behind config flag.
- New orchestration path uses separate launch profile.
- If integration fails, switch launch arg back to direct runtime path.

## Success Definition

- One-command startup for tokenizer + server.
- Stable LLM API boundary (native OpenAI or proxy-mapped).
- LangGraph owns memory/tool policy outside ROS.
- ROS node becomes thin transport/controller with preserved interrupt behavior.
