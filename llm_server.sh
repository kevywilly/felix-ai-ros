#!/usr/bin/env bash
# Serve Nemotron-3-Nano-4B via llama-server on the Jetson GPU, exposing an
# OpenAI-compatible API at http://<jetson>:8080/v1 (host networking, so reachable
# from ROS nodes and Foxglove on the same network).
#
#   ./llm_download.sh   # once, fetches the GGUF to /data/models/llm
#   ./llm_server.sh            # serve the default model (gemma)
#   ./llm_server.sh nemotron   # serve nemotron instead
#
# First arg picks the model (gemma|nemotron, default gemma). Anything after it is
# passed straight through to llama-server.
#
# Test:
#   curl http://localhost:8080/v1/chat/completions -H 'Content-Type: application/json' \
#     -d '{"model":"gemma-4-E4b","messages":[{"role":"user","content":"hi"}]}'
#
# Overridable via env: FELIX_LLM_DIR, LLM_MODEL, LLM_ALIAS, LLM_PORT, LLM_CTX,
# LLM_NGL, LLM_FA (flash-attn: on|off|auto), LLM_REASONING (e.g. deepseek|none).
#
# Any extra CLI args are passed straight through to llama-server, AFTER the
# defaults — so they override (llama-server takes the last occurrence of a flag):
#   ./llm_server.sh nemotron --temp 0.6 --top-p 0.95 --top-k 20
#   ./llm_server.sh gemma --port 9090 --ctx-size 16384 --parallel 2
#   ./llm_server.sh --help        # list every llama-server flag
#!/usr/bin/env bash
set -euo pipefail

DEST="${FELIX_LLM_DIR:-/data/models/llm}"
HOST="${LLM_HOST:-0.0.0.0}"
PORT="${LLM_PORT:-8080}"
CTX="${LLM_CTX:-8192}" # Back to your original modest context size
NGL="${LLM_NGL:-999}"  # Offload all layers to GPU

# Default to gemma as you had it
MODEL_NAME="gemma"
case "${1:-}" in
  gemma|nemotron) MODEL_NAME="$1"; shift ;;
  -*|"") ;; 
  *) echo "ERROR: unknown model '$1' (expected gemma|nemotron)." >&2; exit 1 ;;
esac

case "$MODEL_NAME" in
  gemma)
    MODEL_FILE="gemma-4-E4B-it-Q4_K_M.gguf"
    ALIAS="gemma-4-E4b"
    ;;
  nemotron)
    MODEL_FILE="NVIDIA-Nemotron3-Nano-4B-Q4_K_M.gguf"
    ALIAS="nemotron-3-nano-4b"
    ;;
esac

MODEL="${LLM_MODEL:-$DEST/$MODEL_FILE}"
ALIAS="${LLM_ALIAS:-$ALIAS}"

if [ ! -f "${MODEL}" ]; then
  echo "ERROR: model not found: $MODEL" >&2
  exit 1
fi

MODEL="$(realpath "$MODEL")"

ARGS=(
  --model "$MODEL"
  --host "$HOST"
  --port "$PORT"
  --ctx-size "$CTX"
  --n-gpu-layers "$NGL"
  --jinja        # Back to your original template fallback
  --alias "$ALIAS"
  --tools all    # Restores native MCP schema conversion
  --flash-attn true
)

echo "Serving: $MODEL (alias=$ALIAS)"
exec llama-server "${ARGS[@]}" "$@"
