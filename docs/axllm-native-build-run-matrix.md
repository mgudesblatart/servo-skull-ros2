# AXLLM Native Build and Runtime Matrix

Date: 2026-03-22

## Scope

This document captures:

1. Known-good native build paths for ax-llm.
2. Why the aarch64 cross script fails on Raspberry Pi.
3. Why tokenizer can look healthy but still fail axllm init.
4. A deterministic bring-up checklist for `axllm serve`.

## Key Findings

- The Qwen model config in `~/models/Qwen2.5-1.5B-Instruct/config.json` is not the primary build blocker.
- `build_axcl_aarch64.sh` downloads an x86_64-host cross-compiler, which does not execute on Pi/aarch64 host.
- `axllm serve` startup failure at tokenizer stage can happen even when TCP checks pass, if required tokenizer API endpoints are not actually responding with expected payloads.

## Build Matrix

### A) Raspberry Pi (aarch64 host) native AXCL build

Recommended approach:

- Do not use `build_axcl_aarch64.sh` on Pi host.
- Build with system compiler and system AXCL SDK.

Prerequisites:

- `cmake`, `make`, `g++`, `gcc`
- `axcl-smi` on PATH
- `/usr/include/axcl`
- `/usr/lib/axcl`

Command flow:

```bash
cd ~/projects/ax-llm
mkdir -p build_pi_native
cd build_pi_native
cmake -S .. -B . \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=./install \
  -DBUILD_AX650=OFF \
  -DBUILD_AXCL=ON
make -j"$(nproc)"
make install
```

Expected output binary:

- `build_pi_native/axllm`
- `build_pi_native/install/bin/axllm`

### B) Linux x86_64 host native AXCL build

Use either:

- `build_axcl_x86.sh`
- or explicit CMake command with downloaded AXCL x86 package.

### C) aarch64 cross-compile from x86_64 host only

Use `build_axcl_aarch64.sh` only from x86_64 Linux host.

Reason:

- It downloads `gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu`, which is an x86_64 executable toolchain.

## Why `build_axcl_aarch64.sh` fails on Pi

Failure signature in CMake configure logs:

- `Syntax error: "(" unexpected` from `aarch64-none-linux-gnu-gcc`.

Root cause:

- The compiler binary itself is x86_64 and cannot run on aarch64 host.

## Runtime Bring-Up Checklist (`axllm serve`)

### 1) Validate model config paths

Ensure these keys in model `config.json` resolve correctly from model directory:

- `template_filename_axmodel`
- `filename_post_axmodel`
- `filename_tokens_embed`
- `post_config_path`
- `url_tokenizer_model`
- `devices`

### 2) Start tokenizer first

Example:

```bash
cd ~/models/Qwen2.5-1.5B-Instruct
source ~/projects/venv-servo-skull/bin/activate
python qwen2.5_tokenizer_uid.py
```

### 3) Verify tokenizer protocol endpoints (not only TCP)

Check at least:

- `GET /get_uid`
- encode/decode endpoints required by tokenizer client

A plain root curl or open port check is insufficient.

### 4) Start server

```bash
cd ~/projects/ax-llm/build_pi_native
./axllm serve ~/models/Qwen2.5-1.5B-Instruct --port 8000
```

### 5) Confirm server API shape

If using upstream `axllm serve`, validate OpenAI-style routes:

- `/v1/models`
- `/v1/chat/completions`

If using vendor `main_api_axcl_aarch64`, validate AXERA routes:

- `/api/chat`
- `/api/reset`
- `/api/stop`

## Why tokenizer may still fail despite `curl 127.0.0.1:12345`

Possible causes:

1. Curl checked only socket/root endpoint, not required tokenizer methods.
2. Tokenizer process running with partial dependency behavior and serving minimal endpoints only.
3. URL mismatch in config (e.g. stale config path, wrong model dir used at runtime).
4. Different server process is bound to the same port.

## Fast Diagnostic Commands

```bash
# which axllm are we running
which axllm

# check binary architecture
file ./axllm

# verify tokenizer owner PID
ss -ltnp | grep 12345

# print model config in use
cat ~/models/Qwen2.5-1.5B-Instruct/config.json

# run serve with explicit model dir and watch first 30 log lines
./axllm serve ~/models/Qwen2.5-1.5B-Instruct --port 8000
```

## Recommended Next Step

Create one startup script that:

1. Starts tokenizer.
2. Probes tokenizer protocol endpoints.
3. Starts `axllm serve`.
4. Probes final API readiness.
5. Fails fast with explicit diagnostics when any stage is not healthy.
