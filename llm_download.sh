#!/usr/bin/env bash
# Download a Nemotron-3-Nano-4B GGUF directly over HTTPS into the /data volume
# (persistent, NOT in the image). Uses curl against the HF CDN and deliberately
# avoids the huggingface_hub/Xet client, which hangs on this Jetson. Resumable
# (curl -C -) and idempotent (skips when the local size already matches remote).
#
#   ./llm_download.sh                 # NVIDIA official Q4_K_M (default, recommended on 8 GB)
#   LLM_QUANT=Q5_K_M LLM_REPO=unsloth/NVIDIA-Nemotron-3-Nano-4B-GGUF ./llm_download.sh
#
# Q4_K_M ~2.5 GB, Q5_K_M ~3.0 GB. The official nvidia/ repo only ships Q4_K_M;
# use the unsloth/ repo for the full quant range.
set -euo pipefail

REPO="${LLM_REPO:-nvidia/NVIDIA-Nemotron-3-Nano-4B-GGUF}"
QUANT="${LLM_QUANT:-Q4_K_M}"
DEST="${FELIX_LLM_DIR:-/data/models/llm}"
REV="${LLM_REVISION:-main}"

mkdir -p "$DEST"

# Resolve the exact GGUF filename for this quant via the HF API (public, no auth).
# Repos differ on hyphenation (nvidia: 'Nemotron3-Nano-4B-...', unsloth:
# 'Nemotron-3-Nano-4B-...'), so match on the quant suffix rather than hardcoding.
FILENAME="$(python3 - "$REPO" "$QUANT" <<'PY'
import json, sys, urllib.request
repo, quant = sys.argv[1], sys.argv[2]
req = urllib.request.Request(f"https://huggingface.co/api/models/{repo}",
                             headers={"User-Agent": "curl/8"})
api = json.load(urllib.request.urlopen(req, timeout=30))
ggufs = [s["rfilename"] for s in api.get("siblings", [])
         if s["rfilename"].lower().endswith(".gguf")]
match = [f for f in ggufs if quant.lower() in f.lower()]
if not match:
    sys.stderr.write(f"No *{quant}*.gguf in {repo}. Available:\n  " + "\n  ".join(ggufs) + "\n")
    sys.exit(1)
# shortest match avoids picking split/variant names (e.g. UD-..-XL) over the plain quant
print(sorted(match, key=len)[0])
PY
)"

URL="https://huggingface.co/${REPO}/resolve/${REV}/${FILENAME}"
OUT="${DEST}/${FILENAME}"

# Remote size from the final CDN response (HEAD follows the 302 redirect; take the
# last Content-Length, which is the real file, not the 302 body).
REMOTE_SIZE="$(curl -sIL "$URL" | awk 'tolower($1)=="content-length:"{n=$2} END{gsub(/\r/,"",n); print n}')"

echo "Repo:  $REPO  (quant: $QUANT)"
echo "File:  $FILENAME"
echo "Dest:  $OUT"

if [ -f "$OUT" ] && [ -n "$REMOTE_SIZE" ] && [ "$(stat -c%s "$OUT")" = "$REMOTE_SIZE" ]; then
  echo "Already complete ($REMOTE_SIZE bytes) — skipping download."
else
  echo "Downloading${REMOTE_SIZE:+ ($REMOTE_SIZE bytes)}..."
  curl -L --fail --retry 5 --retry-delay 3 -C - -o "$OUT" "$URL"
fi

echo
echo "Done:"
ls -lh "$OUT"
echo
echo "Next:  ./llm_server.sh"
