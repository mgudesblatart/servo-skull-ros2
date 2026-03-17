#!/usr/bin/env bash
set -euo pipefail

# Custom launcher for AXERA model-pack runtime (main_axcl_aarch64)
#
# Why this script exists:
# - Lets you keep args in English and one place
# - Lets you override settings via env vars without editing model files
# - Makes system prompt behavior explicit
#
# Usage:
#   ./scripts/run_qwen2_5_axcl_custom.sh
#   MODEL_DIR=/path/to/Qwen2.5-1.5B-Instruct ./scripts/run_qwen2_5_axcl_custom.sh
#   SYSTEM_PROMPT_FILE=/path/to/prompt.txt ./scripts/run_qwen2_5_axcl_custom.sh
#
# Important:
# - If your ROS node also prepends a system prompt, disable one side to avoid double prompts.

MODEL_DIR="${MODEL_DIR:-/home/murray/models/Qwen2.5-1.5B-Instruct}"
TOKENIZER_URL="${TOKENIZER_URL:-http://127.0.0.1:12345}"
DEVICE_ID="${DEVICE_ID:-0}"
LIVE_PRINT="${LIVE_PRINT:-1}"
USE_MMAP_EMBED="${USE_MMAP_EMBED:-1}"
AXMODEL_NUM="${AXMODEL_NUM:-28}"
TOKENS_EMBED_NUM="${TOKENS_EMBED_NUM:-151936}"
TOKENS_EMBED_SIZE="${TOKENS_EMBED_SIZE:-1536}"
KVCACHE_PATH="${KVCACHE_PATH:-}"

# System prompt precedence:
# 1) SYSTEM_PROMPT env var (highest)
# 2) SYSTEM_PROMPT_FILE contents
# 3) empty string (no precomputed runtime system prompt)
SYSTEM_PROMPT="${SYSTEM_PROMPT:-}"
SYSTEM_PROMPT_FILE="${SYSTEM_PROMPT_FILE:-}"
if [[ -z "${SYSTEM_PROMPT}" && -n "${SYSTEM_PROMPT_FILE}" && -f "${SYSTEM_PROMPT_FILE}" ]]; then
  SYSTEM_PROMPT="$(cat "${SYSTEM_PROMPT_FILE}")"
fi

BIN="${MODEL_DIR}/main_axcl_aarch64"
[[ -x "${BIN}" ]] || { echo "ERROR: missing executable: ${BIN}"; exit 1; }

CMD=(
  "${BIN}"
  --template_filename_axmodel "${MODEL_DIR}/qwen2.5-1.5b-ctx-ax650/qwen2_p128_l%d_together.axmodel"
  --axmodel_num "${AXMODEL_NUM}"
  --url_tokenizer_model "${TOKENIZER_URL}"
  --filename_post_axmodel "${MODEL_DIR}/qwen2.5-1.5b-ctx-ax650/qwen2_post.axmodel"
  --filename_tokens_embed "${MODEL_DIR}/qwen2.5-1.5b-ctx-ax650/model.embed_tokens.weight.bfloat16.bin"
  --tokens_embed_num "${TOKENS_EMBED_NUM}"
  --tokens_embed_size "${TOKENS_EMBED_SIZE}"
  --use_mmap_load_embed "${USE_MMAP_EMBED}"
  --live_print "${LIVE_PRINT}"
  --devices "${DEVICE_ID}"
)

if [[ -n "${SYSTEM_PROMPT}" ]]; then
  CMD+=( --system_prompt "${SYSTEM_PROMPT}" )
fi

if [[ -n "${KVCACHE_PATH}" ]]; then
  mkdir -p "${KVCACHE_PATH}"
  CMD+=( --kvcache_path "${KVCACHE_PATH}" )
fi

echo "[run_qwen2_5_axcl_custom] MODEL_DIR=${MODEL_DIR}"
echo "[run_qwen2_5_axcl_custom] TOKENIZER_URL=${TOKENIZER_URL}"
echo "[run_qwen2_5_axcl_custom] DEVICE_ID=${DEVICE_ID}"
if [[ -n "${SYSTEM_PROMPT}" ]]; then
  echo "[run_qwen2_5_axcl_custom] system prompt enabled (${#SYSTEM_PROMPT} chars)"
else
  echo "[run_qwen2_5_axcl_custom] system prompt disabled in runtime"
fi

exec "${CMD[@]}"
