# Servo Skull Development Tasks

**Date**: 2026-03-23
**Project Status**: Task 8 (State Machine) complete; Task 9 (Context & Resilience) in planning phase.

## Completed Work

### Task 8: FSM State Machine Implementation ✅
- All state transitions implemented (General, Eye, LLM, Speech)
- Interrupt handling (HALT/HOLD tokens)
- STT gating during SPEAKING state
- Echo suppression (content-aware + time-based)
- LLM failure recovery with status publishing + watchdog
- Integration with native AXLLM backend

**Key Files**:
- `src/skull_control_node/skull_control_node/skull_control_bt_node.py`
- `src/skull_control_node/skull_control_node/llm_agent_http_node.py`
- `src/servo_skull_launch/launch/test_full_audio_pipeline.launch.py`
- `src/servo_skull_launch/launch/axllm_native_backend.launch.py`

**Validated**: Architecture complete. Syntax-checked on development machine. Lives tests on Pi pending.

---

## Active Work: Task 9 - Context & Resilience Hardening

### 9.1: Live Validation of LLM Recovery on Pi
**Status**: PENDING (code complete, runtime validation needed)

**Goal**: Confirm LLM failure recovery hardenings (status publishing + 75s watchdog) work in production on Raspberry Pi.

**What was done**:
- Modified `llm_agent_http_node.py` to publish `/llm_agent/status` events
- Modified `skull_control_bt_node.py` to subscribe and force FSM recovery from THINKING state
- Added 75-second THINKING stall watchdog as fallback

**What to do**:
1. On Pi: `colcon build --packages-select skull_control_node`
2. Source overlay: `source install/setup.bash`
3. Launch pipeline: `ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py start_axllm_native_backend:=true`
4. In separate terminal, monitor: `ros2 topic echo /skull_control/state_transition`
5. Intentionally trigger context_full (feed very long input to LLM)
6. Observe: FSM should exit THINKING within 75s via either status callback OR watchdog timeout
7. Check logs for transition events: look for `llm_failure_*` or `thinking_stall_timeout` events

**Success Criteria**:
- ✅ BT never stuck in THINKING for > 75 seconds
- ✅ Both recovery paths appear in production logs
- ✅ FSM able to send follow-up LLM requests after recovery

**Blockers**: None (code is ready)

---

### 9.2: Message Windowing for Context Management
**Status**: NOT STARTED

**Goal**: Keep conversation history to recent ~4 turns + summary to avoid rapid context exhaustion.

**Why**: Current Qwen3-1.7B pack has finite KV cache budget. As history grows, usable input space shrinks. Windowing keeps only recent turns + summary of older ones.

**Design**:
- Create `src/skull_control_node/skull_control_node/llm_conversation_buffer.py`
- Implement sliding window: system_prompt + summary text + last 2-4 turns max
- Track estimated remaining KV budget in real-time
- On each new turn, discard oldest if window is full
- Before discarding, generate 1-2 sentence summary to preserve context

**Example History State**:
```
[System]: You are a friendly robot. Be concise.
[Summary]: Earlier discussion covered user's hobbies (gardening, music). User prefers short responses.
[Turn 1]: User: "What's your name?" | Assistant: "I'm Servo!"
[Turn 2]: User: "Do you know about plants?" | Assistant: "Yes, I know gardening basics."
[Turn 3]: User: "Tell me about fertilizer" | Assistant: [current response]
```

**Files to Create/Modify**:
- `src/skull_control_node/skull_control_node/llm_conversation_buffer.py` (NEW)
  - Class: `ConversationBuffer(max_turns=4, max_window_tokens=1500)`
  - Methods:
    - `add_turn(role, text, summary=None)` — add user/assistant turn
    - `get_history_for_prompt()` — return formatted history for LLM input
    - `get_remaining_budget()` — estimate tokens left before next reset needed
    - `should_reset()` — return True if < 200 tokens remaining
    - `generate_reset_summary()` — create concise summary of discarded turns
- Modify: `src/skull_control_node/skull_control_node/skull_control_bt_node.py`
  - Import and initialize `ConversationBuffer` in FSM setup
  - In LLM output handling, add turn to buffer (call `add_turn()`)
  - Monitor buffer budget; emit `context_budget_warning` if < 25% remaining

**Success Criteria**:
- ✅ Conversation sustains 50+ turns without context_full error
- ✅ Quality of summary-based context is acceptable (user doesn't notice degradation)
- ✅ Budget warnings appear in logs before resets become necessary

**Estimated Effort**: 3-4 hours (design review, implementation, smoke test)

---

### 9.3: History Compression
**Status**: NOT STARTED

**Goal**: Reduce storage footprint by storing summaries instead of full JSON response objects.

**Why**: Current conversation history stores full LLM response objects (~2-5 KB each). With 50+ turns, this is 100-250 KB per session. Summarizing to text reduces this 10-20x.

**Changes**:
- In `llm_agent_http_node.py` and any persistent storage:
  - Stop storing: `thoughts`, `tool_calls`, `raw_output`, `full_response_object`
  - Keep storing: `user_text`, `assistant_summary_text` (< 100 chars if possible)
- In conversation buffer (Task 9.2):
  - When discarding old turns, extract assistant text only, discard JSON metadata

**Example**:
```python
# Before
conversation.append({
    "user": "What's the capital of France?",
    "assistant": {
        "thoughts": "User asking geography question...",
        "content": "The capital of France is Paris.",
        "tool_calls": [],
        "final_output": "The capital of France is Paris.",
        "tokens_used": 47
    }
})

# After
conversation.append({
    "user": "What's the capital of France?",
    "assistant": "The capital of France is Paris."
})
```

**Files to Modify**:
- `src/skull_control_node/skull_control_node/llm_agent_http_node.py`
  - Trim response before adding to any history/buffer
  - Extract `final_output` or `content` field only
- `src/skull_control_node/skull_control_node/llm_conversation_buffer.py` (if created)
  - Ensure `add_turn()` stores text only, not JSON

**Success Criteria**:
- ✅ Per-turn storage < 200 bytes on average (vs. 2-5 KB current)
- ✅ No perceptible loss in conversation quality

**Estimated Effort**: 1-2 hours

---

### 9.4: Proactive Context Reset Policy
**Status**: NOT STARTED

**Goal**: Reset conversation thread before context_full errors occur, with user awareness.

**Why**: Waiting for context_full is reactive and causes crashes/stalls. Proactive resets are smoother.

**Design**:
- Conversation buffer (Task 9.2) exposes `should_reset()` flag when budget < 200 tokens
- When flag is True:
  1. Publish `context_reset_requested` event on new topic
  2. ROS node receives event and:
     - Generates final summary of current session
     - Publishes to TTS: "Let me start fresh with what I know."
     - Calls LLM reset endpoint with new system prompt + summary
     - Publishes `context_reset_complete` event
  3. FSM transitions to IDLE for a moment, then resumes

**New Topics**:
- `/skull_control/context_reset_requested` — emitted when budget exhausted
- `/skull_control/context_reset_complete` — emitted after reset done

**Files to Modify**:
- `src/skull_control_node/skull_control_node/skull_control_bt_node.py`
  - Subscribe to conversation buffer budget warnings
  - Emit request + handle completion
  - Brief IDLE pause during reset
- `src/skull_control_node/skull_control_node/llm_agent_http_node.py`
  - Listen for reset requests
  - Clear/reseed conversation history with system prompt + summary seed

**Success Criteria**:
- ✅ Extended conversations (50+ turns) never trigger context_full crashes
- ✅ Resets happen transparently (no jarring interruptions)
- ✅ User is aware (TTS notification or visual indicator)
- ✅ Context after reset still maintains awareness of prior conversation

**Estimated Effort**: 2-3 hours

---

### 9.5: Optional Model Pack Rebuild (DEFERRED)
**Status**: LOWER PRIORITY

**Goal**: Increase effective context window by rebuilding the current Qwen model pack with larger prefill groups.

**Why**: Current model pack (p128) has max input of ~1024 tokens initially, shrinking as history grows. Rebuilding with p256 or p512 would double or quadruple this.

**What's involved**:
- Use `ax-llm` Pulsar2 conversion workflow
- Command template: `pulsar2 convert --model_name <current-model> --kv_cache_len 2047 --prefill_len 256 --last_kv_cache_len [512/1024/...]`
- Rebuild takes 30-60 minutes on development machine
- Results in new `.axmodel` files in model pack directory
- Requires updating config.json to reference new files

**Decision Point**: Only pursue if:
- Tasks 9.2-9.4 (windowing, history compression, proactive reset) prove insufficient
- Business requirement emerges for continuous multi-hour conversations without resets

**Status**: Capability documented; not planned unless above conditions met

---

## Quick Reference: What Works Now

✅ **FSM State Machine**: All transitions, timeouts, interrupt handling
✅ **Native AXLLM Backend**: Integration, startup scripts, tokenizer workaround
✅ **LLM Failure Recovery**: Status publishing + watchdog (code complete, Pi validation pending)
✅ **STT Gating During Speech**: Implemented
✅ **Echo Suppression**: Content-aware + time-based gates active

🟡 **Live Runtime Validation on Pi**, **Message Windowing**, **History Compression**, **Proactive Reset**: Planned, not started

---

## Launch Commands

### Full Pipeline with AXLLM Native Backend
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch servo_skull_launch test_full_audio_pipeline.launch.py \
  start_axllm_native_backend:=true
```

### Monitor FSM Transitions (separate terminal)
```bash
ros2 topic echo /skull_control/state_transition
```

### Monitor LLM Status (separate terminal)
```bash
ros2 topic echo /llm_agent/status
```

### Test STT Input During Speech
```bash
ros2 topic pub --once /speech_to_text/transcript std_msgs/String "data: 'test message'"
```

### Test Interrupt (HALT)
```bash
ros2 topic pub --once /speech_to_text/transcript std_msgs/String "data: 'HALT'"
```

---

## Notes for Continuation

- Pi bring-up is the immediate bottleneck; no code blockers remain
- Context windowing (Task 9.2) is highest ROI after Pi validation
- History compression (Task 9.3) is low-risk cleanup work
- Proactive reset (Task 9.4) integrates cleanly once windowing is working
- Model pack rebuild (9.5) is opt-in based on business case; current architecture supports it cleanly

---

**Last Updated**: 2026-03-29
**Next Session Focus**: Live validation on Pi + context windowing design
