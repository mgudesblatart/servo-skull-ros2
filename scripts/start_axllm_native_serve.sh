#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/start_axllm_native_serve.sh [options]

Options:
  --model-dir <path>            Model directory (default: /home/murray/models/Qwen3-1.7B)
  --port <port>                 HTTP port for axllm serve (default: 8081)
  --axllm-bin <path>            axllm binary path (default: /home/murray/projects/ax-llm/build_native/axllm)
  --config-path <path>          Model config path (default: <model-dir>/config.json)
  --help                        Show this help and exit
EOF
}

MODEL_DIR="/home/murray/models/Qwen3-1.7B"
PORT="8081"
AXLLM_BIN="/home/murray/projects/ax-llm/build_native/axllm"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model-dir)
      MODEL_DIR="${2:-}"
      shift 2
      ;;
    --port)
      PORT="${2:-}"
      shift 2
      ;;
    --axllm-bin)
      AXLLM_BIN="${2:-}"
      shift 2
      ;;
    --config-path)
      CONFIG_PATH="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --*)
      echo "[axllm-start] ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      echo "[axllm-start] ERROR: positional arguments are no longer supported. Use --model-dir/--port/..." >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${CONFIG_PATH}" ]]; then
  CONFIG_PATH="${MODEL_DIR}/config.json"
fi

if [[ -z "${MODEL_DIR}" || -z "${PORT}" || -z "${AXLLM_BIN}" ]]; then
  echo "[axllm-start] ERROR: one or more required option values are empty" >&2
  usage >&2
  exit 2
fi

PREFLIGHT_SCRIPT="${SCRIPT_DIR}/axllm_native_tokenizer_preflight.sh"

if [[ ! -x "${AXLLM_BIN}" ]]; then
  echo "[axllm-start] ERROR: axllm binary not executable: ${AXLLM_BIN}" >&2
  exit 1
fi

if [[ ! -f "${PREFLIGHT_SCRIPT}" ]]; then
  echo "[axllm-start] ERROR: preflight script not found: ${PREFLIGHT_SCRIPT}" >&2
  exit 1
fi

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "[axllm-start] ERROR: config not found: ${CONFIG_PATH}" >&2
  exit 1
fi

bash "${PREFLIGHT_SCRIPT}" "${MODEL_DIR}" "${CONFIG_PATH}"

echo "[axllm-start] launching: ${AXLLM_BIN} serve ${MODEL_DIR} --port ${PORT}"
exec "${AXLLM_BIN}" serve "${MODEL_DIR}" --port "${PORT}"
