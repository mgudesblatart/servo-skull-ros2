# Tool Registry and Contract Evolution Plan

Date: 2026-04-27

## Goal

Replace duplicated, manually maintained tool instructions in prompts with a single runtime tool registry that drives:
- prompt tool context,
- contract hints,
- tool validation,
- tool execution.

This keeps JSON-first reliability while making future tools (for example task-list operations) easy to add without prompt drift.

## Current Pain Points

- Tool context currently appears in multiple places (system prompt + JSON contract reminder).
- Adding a new tool requires synchronized edits across prompt/config/parser assumptions.
- Drift risk increases as tool count grows.
- Runtime currently expects a narrow tool shape centered around `say_phrase`.

## Desired End State

A single source of truth for tools in code, with generated prompt context and validated runtime execution.

### Core properties

- JSON-first output remains canonical.
- Tool metadata is defined once in a registry.
- Prompt tool section is generated from the registry.
- Parser normalizes tool calls into one internal canonical form.
- Executor validates args against schema before dispatch.

## Proposed Architecture

## 1) Tool Registry (single source of truth)

Represent each tool as metadata + handler binding.

Suggested fields per tool:
- `name`: stable tool identifier.
- `description`: short behavior summary.
- `args_schema`: JSON-schema-like dict for runtime validation.
- `enabled`: dynamic flag or mode-based predicate.
- `usage_notes`: optional prompt hints.
- `handler`: callable reference (dispatch target).

Example conceptual entry:
- name: `say_phrase`
- args_schema: `{ "type": "object", "properties": { "msg": {"type": "string"} }, "required": ["msg"] }`

## 2) Prompt Builder from Registry

Introduce a prompt-builder layer in `llm_agent_http_node` that composes:
- base persona/system instructions,
- generated tool section (from enabled tools),
- stable JSON contract instructions.

Principles:
- No duplicated hand-edited tool docs in YAML and code.
- Keep the final prompt compact (summaries, not verbose manuals).
- Support mode-aware tool exposure (minimal mode vs full mode).

## 3) Contract Shape (canonical internal form)

Canonical tool call shape should be:
- `{ "name": "tool_name", "args": { ... } }`

Response contract (externally encouraged):
- `thoughts: string`
- `tool_calls: array` of canonical calls

Compatibility:
- Accept and normalize legacy inline forms (for example `{"say_phrase": {"msg": "..."}}`).
- Keep compatibility during migration; remove later only when stable.

## 4) Validation and Execution Layer

Execution flow:
1. Parse model output (existing robust JSON-first parser).
2. Normalize each tool call into canonical `{name,args}`.
3. Validate:
   - known tool name,
   - args match schema.
4. Dispatch to handler.
5. Emit structured status/log events for failures.

Failure policy:
- Unknown/invalid tool call is non-fatal to process stability.
- Record warning and continue with remaining valid calls.

## 5) Versioning and Compatibility

Add contract version awareness to reduce migration risk.

Options:
- explicit `contract_version` key in model output guidance, or
- inferred default version internally if key absent.

Recommendation:
- Start with inferred `v1` (current), support `v2` canonical tool calls.
- Keep parser compatibility shims until model behavior stabilizes.

## Migration Plan (Phased)

## Phase 0: Prep (No behavior change)

- Add tool registry module and schemas.
- Add prompt builder that can render current `say_phrase` tool docs.
- Keep existing prompt text path as fallback.

Acceptance:
- Runtime behavior unchanged.
- Generated tool section matches current semantics.

## Phase 1: Canonicalization Layer

- Add tool-call normalizer in parser/runtime path.
- Internally convert legacy and emergent formats to `{name,args}`.
- Keep existing `extract_say_phrase_calls` behavior intact for compatibility.

Acceptance:
- Existing `say_phrase` flows still pass.
- New tests for normalization pass.

## Phase 2: Registry-Driven Prompting

- Replace duplicated manual tool instructions with generated registry section.
- Keep persona/system text in YAML, but remove hand-maintained per-tool duplication.

Acceptance:
- Prompt content remains stable and compact.
- No regression in JSON parse rates or speech behavior.

## Phase 3: Add First Non-Speech Tool(s)

Candidate tools:
- `list_tasks`
- `add_task`

Implement handlers with strict schema validation and safe failure behavior.

Acceptance:
- Tool execution is deterministic and logged.
- Unknown/invalid calls do not destabilize the node.

## Phase 4: Cleanup

- Remove obsolete legacy guidance and redundant helper paths when metrics show stability.
- Retain compatibility shim only if model still emits legacy format at non-trivial frequency.

## Testing Strategy

Unit tests:
- parser normalization for multiple tool-call shapes,
- schema validation pass/fail cases,
- executor dispatch behavior,
- unknown tool and invalid args handling.

Integration tests:
- end-to-end prompt -> model output -> parsed tool call -> handler side effect,
- mixed valid/invalid tool_calls in same response,
- backward compatibility with legacy `say_phrase` payload.

Runtime checks:
- parse success rate,
- invalid-tool and invalid-args counters,
- fallback-contract usage rate,
- tool execution latency/error counts.

## Risks and Mitigations

Risk: Prompt bloat from auto-generated tool docs.
- Mitigation: concise generated summaries and mode-based tool filtering.

Risk: Model emits mixed legacy/new tool formats.
- Mitigation: normalization compatibility layer + metrics-driven cleanup.

Risk: Unsafe side effects from future tools.
- Mitigation: strict schemas, allowlist registry, explicit handler boundaries.

## Suggested Initial File Touch Points

- `src/skull_control_node/skull_control_node/llm_agent_http_node.py`
  - prompt composition,
  - tool execution integration.
- `src/skull_control_node/skull_control_node/response_parser.py`
  - tool-call normalization support.
- New module (suggested):
  - `src/skull_control_node/skull_control_node/tool_registry.py`
  - `src/skull_control_node/skull_control_node/tool_executor.py`
- `src/skull_control_node/configs/http_servo_skull.yaml`
  - reduce duplicated hardcoded tool contract details once generation is in place.

## Decision Summary

- Keep JSON-first as canonical output contract.
- Add registry-driven tool metadata and generated prompt tool context.
- Normalize legacy/emergent tool-call shapes into one internal canonical form.
- Roll out incrementally with compatibility and instrumentation.

This approach minimizes operational risk while making future tool expansion practical.
