---
title: "feat: Nemotron-3-Nano-4B on-device LLM via llama.cpp (CUDA)"
type: feat
status: active
date: 2026-06-23
---

# feat: Nemotron-3-Nano-4B on-device LLM via llama.cpp (CUDA)

## Summary

Run NVIDIA's `Nemotron-3-Nano-4B` as an on-device LLM on Felix's Jetson Orin Nano
(8 GB), serving an OpenAI-compatible HTTP API for ROS/Foxglove clients. The model is a
**hybrid Mamba-2 + Transformer** (`model_type: nemotron_h`), which rules out MLC/NanoLLM
and TensorRT-LLM as practical runtimes on this board — it runs via **llama.cpp built with
CUDA**, using a prebuilt **Q4_K_M GGUF** from NVIDIA's official HF repo. The split is:
**llama.cpp binaries baked into the image**, **GGUF weights on the `/data` volume** (never
in image layers). Three artifacts are already created (Dockerfile tier + two helper
scripts); what remains is the host-side rebuild, weight download, and serve/verify.

This supersedes the user's original ask (Mistral-NeMo-Minitron-8B-Base + TensorRT-LLM),
which was abandoned because an 8B model + TRT-LLM is the wrong fit for an 8 GB Orin Nano
(see Key Technical Decisions and Deferred).

## Problem Frame

Felix runs a full ROS 2 Humble autonomy stack in a custom Docker container on a Jetson
Orin Nano 8 GB (JetPack 6.2 / L4T R36.4.4, CUDA 12.6). We want a local LLM for
on-robot reasoning / command interpretation, reachable over HTTP so ROS nodes can call it
without embedding model code. Constraints that shaped the whole approach:

- **8 GB unified memory**, shared with the OS and the ROS/Nav2 stack → only a 4B-class
  model at INT4 fits with headroom.
- The container has **no Docker socket**, so the runtime install must go through the
  project Dockerfile and a **host-side rebuild** (`docker/build.sh`), not an in-container
  install.
- The chosen model is a **hybrid Mamba-2 reasoning model**, which most Jetson LLM stacks
  (MLC/NanoLLM, and practically TRT-LLM on this board) do not support.

---

## Hardware & Environment (verified facts)

| Item | Value |
|---|---|
| Board | Jetson **Orin Nano 8 GB** (`BLINKA_FORCEBOARD=JETSON_ORIN_NANO`; MemTotal ≈ 7.8 GB) |
| JetPack / L4T | **6.2** / R36.4.4 |
| CUDA / TensorRT | 12.6 / 10.7 |
| CUDA arch | **sm_87** (Ampere) → `CMAKE_CUDA_ARCHITECTURES=87` |
| Base image | `ultralytics/ultralytics:latest-jetson-jetpack6` (ships torch 2.10 / CUDA 12.6) |
| Build | single image `felix-ai-ros:latest` via `docker/build.sh` (`--no-cache --network=host`); **no docker-compose** |
| Persistent volume | host `~/data` → `/data` (already holds `models/huggingface`, `models/mlc`) |
| Networking | `--network host` (API reachable on the LAN) |

**Model:** `nvidia/NVIDIA-Nemotron-3-Nano-4B-GGUF` → file
`NVIDIA-Nemotron3-Nano-4B-Q4_K_M.gguf` (~2.5 GB). `config.json` confirms
`model_type: nemotron_h`, `NemotronHForCausalLM`, `hybrid_override_pattern`,
`mamba_num_heads`, `ssm_state_size` (hybrid Mamba-2), plus `nano_v3_reasoning_parser.py`
(it is a reasoning model that emits `<think>` traces).

---

## Current State (already implemented)

These exist in the working tree (owned `1000:1000`, syntax-checked):

- **`docker/Dockerfile`** — new *Tier 10.5* (between cv2 Tier 10 and app-scripts Tier 11):
  clones + builds llama.cpp with `-DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87
  -DLLAMA_CURL=ON`, installs binaries to `/usr/local`, `ldconfig`. Sets
  `FELIX_LLM_DIR=/data/models/llm`. (No `huggingface_hub` install — weights download is
  direct curl; see U2.) Build args `LLAMA_CPP_REF=b9763` (pinned release),
  `LLAMA_BUILD_JOBS=4`.
- **`llm_download.sh`** (repo root) — downloads the GGUF into `/data/models/llm` via
  **direct curl** over the HF CDN (resolves the exact filename via the HF API, resumable
  `-C -`, size-based skip). Avoids the huggingface_hub/Xet client, which hangs on this
  Jetson. Env: `LLM_REPO`, `LLM_QUANT` (default `Q4_K_M`), `FELIX_LLM_DIR`, `LLM_REVISION`.
- **`llm_server.sh`** (repo root) — launches `llama-server` on `:8080`, all layers on GPU,
  `--jinja`, `--alias nemotron-3-nano-4b`. Env: `LLM_PORT`, `LLM_CTX` (default 8192),
  `LLM_NGL`, `LLM_FA`, `LLM_REASONING`, `LLM_MODEL`.

What remains: **U4** (host rebuild + download + serve + verify) and the optional **U5**
(ROS-side client). These have not been run yet.

---

## Key Technical Decisions

- **KTD1 — Runtime is llama.cpp (CUDA), not MLC/NanoLLM, Ollama, or TensorRT-LLM.**
  The model is hybrid Mamba-2 (`nemotron_h`). MLC/NanoLLM have no real hybrid-Mamba
  support and no prebuilt MLC/AWQ build exists; TRT-LLM on an 8 GB Nano is OOM-prone to
  build and leaves no room for ROS. llama.cpp upstream supports `nemotron_h` (proven by
  the existence of official + community GGUFs), builds small/fast with CUDA, and ships
  `llama-server` with an OpenAI-compatible API — ideal for ROS integration over HTTP.
  Ollama (which wraps llama.cpp) is a viable easy-mode alternative *iff* its bundled engine
  version includes Mamba-2/`nemotron_h`; llama.cpp direct avoids that version risk.
- **KTD2 — Model is 4B Nemotron, not the originally-requested 8B Minitron.** 8B at INT4
  (~4.7 GB) is marginal alongside Nav2 on 8 GB and the engine build tends to OOM. A 4B at
  Q4_K_M (~2.5 GB) fits with headroom. Bonus: Mamba-2 layers have ~constant state memory
  vs. a growing KV cache, so long context is cheap on this RAM-limited board.
- **KTD3 — Quant is Q4_K_M.** Q5_K_M closes ~half the remaining quality gap to FP16 but
  for a 4B that gap is small, while it costs ~+0.5 GB and ~10–15 % tok/s on this
  bandwidth-bound board. Since it is a reasoning model (long generations), tok/s matters.
  Q5 remains a one-line A/B via `LLM_QUANT`/`LLM_REPO` (use the `unsloth/` repo for the
  full quant range — the official `nvidia/` repo only ships Q4_K_M).
- **KTD4 — Binaries in the image, weights on `/data`.** Baking ~2.5 GB+ of weights into
  image layers would bloat every `--no-cache` rebuild; `/data/models/` is the existing
  convention (HF cache + MLC dist already live there) and persists across rebuilds.
- **KTD5 — Conservative build parallelism (`LLAMA_BUILD_JOBS=4`).** The CUDA compile runs
  on the host during `docker/build.sh` — i.e. on the same 8 GB — and nvcc is RAM-hungry;
  the 15 GB swap covers `-j4`. Drop to `-j2` if the build OOMs.

---

## Implementation Units

### U1. Dockerfile Tier 10.5 — build llama.cpp with CUDA (DONE)

- **Goal:** Bake `llama-server`/`llama-cli` (CUDA-enabled) into the image.
- **Files:** `docker/Dockerfile` (Tier 10.5, inserted before Tier 11).
- **Approach:** `git clone --depth=1 --branch ${LLAMA_CPP_REF}` of `ggml-org/llama.cpp`;
  cmake with `GGML_CUDA=ON`, `CMAKE_CUDA_ARCHITECTURES=87`, `LLAMA_CURL=ON`,
  `Release`; `cmake --install --prefix /usr/local`; `ldconfig`; clean up build dir + apt
  lists. Sets `FELIX_LLM_DIR`. No `huggingface_hub` (download is direct curl, U2).
- **Patterns to follow:** the existing cache-ordered "Tier N" comment convention.
- **Verification:** image builds; inside the container `command -v llama-server` resolves
  and `llama-server --version` reports a CUDA build.
- **Test expectation:** none — build-config change; behavior is verified in U4.

### U2. Weight download helper (DONE)

- **Goal:** Fetch the Q4_K_M GGUF to `/data/models/llm` idempotently; allow quant/repo override.
- **Files:** `llm_download.sh`.
- **Approach:** Direct curl over the HF CDN — resolve the exact filename via the HF API
  (`/api/models/<repo>` siblings, match on quant suffix so `nvidia/` vs `unsloth/`
  hyphenation doesn't matter), then `curl -L --fail --retry -C -` to `$FELIX_LLM_DIR`.
  Size-based skip makes re-runs idempotent. No huggingface_hub/Xet (the Xet client hangs
  on this Jetson).
- **Verification:** after run, `/data/models/llm/*Q4_K_M*.gguf` exists at the expected byte
  size (2,837,072,864 for the NVIDIA Q4_K_M).
- **Test expectation:** none — operational script; covered by U4 smoke test.

### U3. llama-server launch helper (DONE)

- **Goal:** Serve the GGUF on the GPU with an OpenAI API for ROS clients.
- **Files:** `llm_server.sh`.
- **Approach:** auto-select newest matching GGUF; `llama-server --n-gpu-layers 999
  --ctx-size 8192 --jinja --host 0.0.0.0 --port 8080 --alias nemotron-3-nano-4b`;
  optional `--flash-attn`/`--reasoning-format` via env.
- **Verification:** server starts, reports layers offloaded to CUDA; `/v1/models` lists
  `nemotron-3-nano-4b`.
- **Test expectation:** none — operational script; covered by U4 smoke test.

### U4. Host rebuild, download, serve & verify (REMAINING)

- **Goal:** Produce a working end-to-end on-device LLM endpoint.
- **Dependencies:** U1, U2, U3.
- **Files:** none (operational); run from the **host**, then inside the container.
- **Approach (host):** `cd ~/felix-ai-ros && ./docker/build.sh` (expect a slow one-time
  CUDA compile; if OOM, rebuild with `--build-arg LLAMA_BUILD_JOBS=2`), then
  `./start-container.sh`. **(container):** `./llm_download.sh` then `./llm_server.sh`.
- **Verification (smoke test):**
  - `curl http://localhost:8080/v1/chat/completions -H 'Content-Type: application/json'
    -d '{"model":"nemotron-3-nano-4b","messages":[{"role":"user","content":"hi"}]}'`
    returns a completion.
  - `tegrastats` (or `jtop`) shows GPU utilization during generation and free RAM stays
    non-trivial with the ROS stack running.
  - Confirm responses include `<think>` traces (reasoning model); note baseline tok/s.
- **Execution note:** first run with the robot on a stand is not required (no motor
  activity), but watch memory pressure if Nav2 is running concurrently.

### U5. ROS-side LLM client (DEFERRED / future)

- **Goal:** A thin ROS node or helper that calls the `:8080` OpenAI endpoint (e.g. a
  command/intent service), with `<think>`-trace handling and a latency budget.
- **Dependencies:** U4.
- **Files:** TBD (likely a new `felix_*` node or a helper in an existing package).
- **Approach:** out of scope for this plan; design once the endpoint is verified and the
  use case (command parsing vs. chat vs. tool-calling) is chosen. Decide reasoning posture
  then (`LLM_REASONING=none` or low effort for snappy commands).
- **Test scenarios:** defined when the node is specified.

---

## Verification (overall)

The feature is complete (through U4) when: the image rebuilds with the llama.cpp CUDA
tier; `./llm_download.sh` lands the Q4_K_M GGUF on `/data`; `./llm_server.sh` serves it
with GPU offload; and the `/v1/chat/completions` smoke test returns a coherent reply while
leaving enough RAM for the ROS stack.

---

## Scope Boundaries

**In scope:** on-device Nemotron-3-Nano-4B via llama.cpp CUDA, image tier + helper
scripts, host rebuild and serve/verify.

### Deferred to Follow-Up Work
- **U5 ROS client** — the actual ROS integration that consumes the endpoint.
- **Q5_K_M A/B** — pull `unsloth` Q5_K_M and compare quality vs. tok/s/RAM.
- **Reasoning-posture tuning** — `LLM_FA`, `LLM_REASONING`, ctx sizing once real latency is measured.
- **Bumping `LLAMA_CPP_REF`** — currently pinned to `b9763`; bump to a newer `bN` tag periodically (re-verify `nemotron_h` support and CLI flag compatibility on bump).

### Explicitly Not Doing
- **Mistral-NeMo-Minitron-8B-Base + TensorRT-LLM** (original ask) — wrong fit for 8 GB
  Orin Nano: TRT-LLM has no clean Jetson pip path (jetson-ai-lab index didn't even resolve
  from the container), the engine build is OOM-prone on-device, and an 8B leaves no room
  for ROS. Also it's a *base* (non-instruct) model.
- **MLC/NanoLLM** for this model — no hybrid-Mamba support.
- **Baking weights into the image** — they live on `/data`.

---

## Risks & Mitigations

- **CUDA compile OOM during host rebuild** → `LLAMA_BUILD_JOBS=4` default + 15 GB swap;
  fall back to `-j2`.
- **llama.cpp version drift** — mitigated by pinning `LLAMA_CPP_REF=b9763` (reproducible
  builds). On a future bump, re-verify `nemotron_h` support and CLI flag forms
  (e.g. `--flash-attn`); `LLM_FA`/`LLM_REASONING` are opt-in to avoid flag breakage.
- **Binary split (this build):** `llama-cli` is interactive-only and rejects `-no-cnv` —
  for a scriptable/non-interactive one-shot use **`llama-completion`** (passing `-no-cnv`
  to `llama-cli` makes it drop into interactive mode and block on stdin, which looks like a
  hang in a non-TTY/background context). Serving is unaffected — `llm_server.sh` uses
  `llama-server`. (GPU validation passed: `-ngl 999` full offload of the hybrid-Mamba
  model works.)
- **GPU / unified-memory contention (8 GB shared)** — the LLM competes for two distinct
  resources: (1) **unified RAM + memory bandwidth (EMC)** with *everything* incl. the
  camera (frame buffers), and (2) **CUDA SMs** specifically with `felix_perception`
  (YOLO/TensorRT) — these time-slice (no MPS/MIG on Jetson), so concurrent run slows both.
  The CSI camera capture/encode uses ISP + NVJPG/NVENC (separate silicon), so it does *not*
  fight the LLM for CUDA cores — only for RAM/bandwidth. Mitigate: keep `LLM_CTX` modest +
  `--parallel 1`, watch `RAM`/`GR3D_FREQ`/`EMC_FREQ` in `tegrastats`/`jtop` (host), consider
  `nvpmodel -m 0`+`jetson_clocks`, and don't run the LLM at full tilt during active
  navigation if all four subsystems (camera + perception + LLM + Nav2) are live.
- **Reasoning latency** — long `<think>` traces slow responses; mitigate with
  `LLM_REASONING=none`/low effort and output caps for command-style use (U5).
- **HF Xet client hangs on this Jetson** — `hf_transfer` is deprecated and the Xet
  download path stalls at 0 progress here. Mitigated by downloading via direct curl over
  the CDN (`llm_download.sh`), which is healthy (verified: 302 → CloudFront, 2.64 GB).

---

## Sources & Research

- Model: `nvidia/NVIDIA-Nemotron-3-Nano-4B-BF16` (source), `…-GGUF` (Q4_K_M); fuller quant
  range at `unsloth/NVIDIA-Nemotron-3-Nano-4B-GGUF`.
- Architecture confirmed from the model's `config.json` (`model_type: nemotron_h`, hybrid
  Mamba-2 keys) and repo files (`modeling_nemotron_h.py`, `nano_v3_reasoning_parser.py`).
- Env facts gathered on-device: L4T R36.4.4 / JetPack 6.2, CUDA 12.6, TRT 10.7,
  torch 2.10, MemTotal ≈ 7.8 GB, `BLINKA_FORCEBOARD=JETSON_ORIN_NANO`.
- llama.cpp: `ggml-org/llama.cpp`, built with `GGML_CUDA=ON`, `CMAKE_CUDA_ARCHITECTURES=87`.
