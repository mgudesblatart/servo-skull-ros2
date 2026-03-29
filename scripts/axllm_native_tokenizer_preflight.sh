#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR="${1:-/home/murray/models/Qwen3-1.7B}"
CONFIG_PATH="${2:-${MODEL_DIR}/config.json}"

if [[ ! -d "${MODEL_DIR}" ]]; then
  echo "[axllm-preflight] ERROR: model dir not found: ${MODEL_DIR}" >&2
  exit 1
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "[axllm-preflight] ERROR: config not found: ${CONFIG_PATH}" >&2
  exit 1
fi

readarray -t parsed < <(python3 - <<'PY' "${CONFIG_PATH}" "${MODEL_DIR}"
import json
import os
import sys

cfg = sys.argv[1]
model_dir = sys.argv[2]
with open(cfg, "r", encoding="utf-8") as f:
    data = json.load(f)

raw = str(data.get("url_tokenizer_model", "")).strip()
if not raw:
    print("", flush=True)
    print("", flush=True)
    sys.exit(0)

if "://" in raw:
    resolved = raw
else:
    resolved = raw if os.path.isabs(raw) else os.path.join(model_dir, raw)

print(raw, flush=True)
print(resolved, flush=True)
PY
)

RAW_URL="${parsed[0]:-}"
RESOLVED_PATH="${parsed[1]:-}"

if [[ -z "${RAW_URL}" ]]; then
  echo "[axllm-preflight] ERROR: config missing url_tokenizer_model" >&2
  exit 1
fi

if [[ "${RAW_URL}" == http://* || "${RAW_URL}" == https://* ]]; then
  echo "[axllm-preflight] ERROR: native axllm expects local tokenizer.txt, not HTTP tokenizer URL." >&2
  echo "[axllm-preflight] Found url_tokenizer_model=${RAW_URL}" >&2
  echo "[axllm-preflight] Fix: export tokenizer.txt and set url_tokenizer_model to tokenizer.txt" >&2
  exit 2
fi

if [[ ! -f "${RESOLVED_PATH}" ]]; then
  echo "[axllm-preflight] ERROR: tokenizer file not found: ${RESOLVED_PATH}" >&2
  exit 3
fi

if ! head -n 1 "${RESOLVED_PATH}" | grep -q '^430 '; then
  echo "[axllm-preflight] ERROR: tokenizer file header does not look like tokenizer.axera export: ${RESOLVED_PATH}" >&2
  exit 4
fi

echo "[axllm-preflight] OK: tokenizer config is native-compatible"
echo "[axllm-preflight] model_dir=${MODEL_DIR}"
echo "[axllm-preflight] config_path=${CONFIG_PATH}"
echo "[axllm-preflight] tokenizer=${RESOLVED_PATH}"
