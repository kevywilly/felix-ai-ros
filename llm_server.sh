#!/usr/bin/env bash
# Serve Nemotron-3-Nano-4B via llama-server on the Jetson GPU, exposing an
# OpenAI-compatible API at http://<jetson>:8080/v1 (host networking, so reachable
# from ROS nodes and Foxglove on the same network).
#
#   ./llm_download.sh   # once, fetches the GGUF to /data/models/llm
#   ./llm_server.sh     # serve it
#
# Test:
#   curl http://localhost:8080/v1/chat/completions -H 'Content-Type: application/json' \
#     -d '{"model":"nemotron-3-nano-4b","messages":[{"role":"user","content":"hi"}]}'
#
# Overridable via env: FELIX_LLM_DIR, LLM_QUANT, LLM_MODEL, LLM_PORT, LLM_CTX,
# LLM_NGL, LLM_FA (flash-attn: on|off|auto), LLM_REASONING (e.g. deepseek|none).
#
# Any extra CLI args are passed straight through to llama-server, AFTER the
# defaults — so they override (llama-server takes the last occurrence of a flag):
#   ./llm_server.sh --temp 0.6 --top-p 0.95 --top-k 20
#   ./llm_server.sh --port 9090 --ctx-size 16384 --parallel 2
#   ./llm_server.sh --help        # list every llama-server flag
set -euo pipefail

DEST="${FELIX_LLM_DIR:-/data/models/llm}"
QUANT="${LLM_QUANT:-Q4_K_M}"
HOST="${LLM_HOST:-0.0.0.0}"
PORT="${LLM_PORT:-8080}"
CTX="${LLM_CTX:-8192}"          # model supports 262144; keep modest on 8 GB shared w/ ROS
NGL="${LLM_NGL:-999}"           # offload all layers to GPU

# Pick the model file: explicit LLM_MODEL, else newest GGUF matching the quant.
MODEL="${LLM_MODEL:-$(ls -t "$DEST"/*"${QUANT}"*.gguf 2>/dev/null | head -1 || true)}"
if [ -z "${MODEL}" ] || [ ! -f "${MODEL}" ]; then
  echo "ERROR: no '*${QUANT}*.gguf' in $DEST — run ./llm_download.sh first." >&2
  exit 1
fi

ARGS=(
  --model "$MODEL"
  --host "$HOST" --port "$PORT"
  --ctx-size "$CTX"
  --n-gpu-layers "$NGL"
  --jinja                       # use the model's embedded chat template
  --alias nemotron-3-nano-4b
  --tools all
)
# Optional flags (only added if requested — keeps compat across llama.cpp versions).
[ -n "${LLM_FA:-}" ]        && ARGS+=( --flash-attn "$LLM_FA" )
[ -n "${LLM_REASONING:-}" ] && ARGS+=( --reasoning-format "$LLM_REASONING" )

echo "Serving: $MODEL"
echo "  OpenAI API: http://${HOST}:${PORT}/v1   (ctx=$CTX, gpu-layers=$NGL)"
echo "  NOTE: this is a reasoning model — responses include <think> traces."
exec llama-server "${ARGS[@]}" "$@"
