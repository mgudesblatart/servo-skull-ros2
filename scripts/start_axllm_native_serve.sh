#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR="${1:-/home/murray/models/Qwen2.5-1.5B-Instruct}"
PORT="${2:-8011}"
AXLLM_BIN="${3:-/home/murray/projects/ax-llm/build_native/axllm}"
CONFIG_PATH="${4:-${MODEL_DIR}/config.json}"

if [[ -z "${CONFIG_PATH}" ]]; then
  CONFIG_PATH="${MODEL_DIR}/config.json"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFLIGHT_SCRIPT="${SCRIPT_DIR}/axllm_native_tokenizer_preflight.sh"

if [[ ! -x "${AXLLM_BIN}" ]]; then
  echo "[axllm-start] ERROR: axllm binary not executable: ${AXLLM_BIN}" >&2
  exit 1
fi

if [[ ! -f "${PREFLIGHT_SCRIPT}" ]]; then
  echo "[axllm-start] ERROR: preflight script not found: ${PREFLIGHT_SCRIPT}" >&2
  exit 1
fi

bash "${PREFLIGHT_SCRIPT}" "${MODEL_DIR}" "${CONFIG_PATH}"

echo "[axllm-start] launching: ${AXLLM_BIN} serve ${MODEL_DIR} --port ${PORT}"
exec "${AXLLM_BIN}" serve "${MODEL_DIR}" --port "${PORT}"
