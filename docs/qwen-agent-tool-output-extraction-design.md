# Qwen-Agent Style Tool-Output Extraction Design

Date: 2026-03-29
Status: Draft
Scope: skull_control HTTP LLM path

## Why this doc exists

Qwen-Agent supports tool usage with two practical strategies:

1. Native structured tool calls from the model service (tool_calls/function_call objects).
2. Text-format tool calls parsed back into structured calls when native tool calls are unavailable.

The servo-skull stack currently relies on text contract parsing with robustness fixes. This document captures a forward-compatible design to adopt the same dual-lane extraction pattern without disrupting the current pipeline.

It also captures project strategy for where tool-calling should live:

1. Harden local extraction/parsing in this repo first.
2. Add native tool-calling support in AXLLM (service/API layer).
3. Do not add tool-calling behavior to AXCL runtime unless a hard runtime limitation forces it.

## Current state in this repo

- LLM entry point: src/skull_control_node/skull_control_node/llm_agent_http_node.py
- Response parsing and fallback contract: src/skull_control_node/skull_control_node/response_parser.py
- Current resilience includes:
  - stripping <think>...</think>
  - typo repair for malformed JSON separators
  - JSON extraction + schema validation
  - fallback wrapper for non-JSON output

## Observed Qwen-Agent pattern

Qwen-Agent effectively normalizes all model output into a single internal representation before the agent loop executes tools.

### Stage A: Preprocess

- Inject tool descriptions and expected format into prompt context.
- Convert conversation and prior tool results into model-compatible format.
- Keep function schema in OpenAI-like structure where possible.

### Stage B: Model output ingestion

- If backend supports native tool calls, collect and merge streamed partial chunks.
- If backend does not support native tool calls, receive text and parse using a known call format.

### Stage C: Postprocess normalization

- Emit a unified internal structure:
  - assistant content/reasoning (optional)
  - function call(s): name + arguments + id

### Stage D: Tool loop execution

- Execute tool(s), append tool-result messages, continue until no tool call is emitted.

## Proposed adaptation for servo-skull

### Design goal

Add structured tool-call support without breaking current text-contract behavior.

### Layering decision (important)

- AXLLM is the correct abstraction layer for native tool-calling protocol behavior.
- AXCL runtime is a low-level execution/runtime layer and should remain focused on model execution concerns.
- This repo should continue to normalize and orchestrate responses so behavior stays stable across backend differences.

### Dual-lane extraction strategy

Lane 1 (preferred): Native tool_calls extraction
- Inspect model response payload for structured tool calls first.
- Normalize each call to internal shape:
  - id: stable call id (or generated id)
  - name: function/tool name
  - arguments: JSON string or object normalized to JSON string

Lane 2 (fallback): Text contract parsing
- Reuse parse_response() flow for content-only responses.
- Keep typo repair and fallback contract for malformed text.

Unified output object (internal)
- thoughts: optional assistant reasoning/content summary
- tool_calls: normalized list of tool calls
- text: optional plain assistant text

## Implementation path

### Phase 0: Local hardening (current repo)

1. Add response adapter in llm_agent_http_node.py
- New helper: extract_native_tool_calls(raw_response) -> list[dict]
- New helper: normalize_tool_call(tc) -> {id, name, arguments}

2. Integrate with existing parser path
- Try native structured extraction first.
- If no tool calls are found, run existing text parser path.

3. Keep speech behavior stable
- Continue using existing say_phrase extraction semantics.
- Never speak parser diagnostics or schema hints.

4. Add tests/fixtures
- Native tool_calls (single call).
- Native tool_calls (parallel calls).
- Mixed output (text + tool_calls).
- Malformed JSON text patterns already seen in production.

### Phase 1: AXLLM contribution path (recommended)

1. Propose an AXLLM API contract for native tool calls
- OpenAI-compatible response shape with message.tool_calls support.
- Stable arguments format and call id semantics.

2. Add AXLLM server-side output mode
- Emit structured tool calls when model output can be interpreted as calls.
- Preserve existing content output for backward compatibility.

3. Add compatibility tests against this repo adapter
- Ensure native AXLLM tool_calls and text fallback both normalize to the same internal shape.

### Phase 2: Optional cleanup

- Once AXLLM native tool calls are stable, reduce prompt-level text-contract pressure.
- Keep parser fallback permanently as resilience path for malformed outputs.

## Non-goals (for this phase)

- Implementing tool-calling logic in AXCL runtime.
- Reworking AXERA runtime internals for protocol-level behavior.
- Full OpenAI tool-calling protocol implementation across all services in one pass.
- Streaming delta merge for partial tool_call chunks in ROS node internals.
- Large prompt/template redesign.

## Risks and mitigations

Risk: Native and text paths diverge in behavior.
- Mitigation: Normalize both paths into one internal object before downstream logic.

Risk: AXLLM upstream path takes time or stalls.
- Mitigation: Keep local Phase 0 path production-ready and independent.

Risk: Arguments shape inconsistency (object vs string).
- Mitigation: Canonicalize arguments to JSON string at normalization boundary.

Risk: Regression in speech output.
- Mitigation: Keep current say_phrase extraction fallback path as safety net.

Risk: Scope creep into AXCL runtime internals.
- Mitigation: Treat AXCL modifications as out-of-scope unless required for core inference correctness/performance.

## Suggested sequence when implementing

1. Complete Phase 0 in this repo.
2. Validate end-to-end with ROS topic pub tests.
3. Draft AXLLM proposal for native tool_calls shape and compatibility contract.
4. Upstream Phase 1 changes to AXLLM.

## Open decisions

- Internal canonical format for arguments:
  - Option A: dict in memory, stringify only at tool boundary
  - Option B: JSON string everywhere (simpler parity with model outputs)

- Whether to support parallel tool calls immediately or serialize first.

Current recommendation: start with serialized execution, keep schema ready for parallel.

## Expected outcome

- Better robustness when model/backend emits structured tool calls.
- Continued resilience for malformed text output.
- Cleaner path toward upstream AXLLM tool-calling support without runtime-layer churn.
