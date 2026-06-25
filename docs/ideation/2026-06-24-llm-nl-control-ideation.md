---
date: 2026-06-24
topic: llm-nl-control
focus: teach named rooms on the map; tell a local LLM to go to them
mode: repo-grounded
---

# Ideation: Natural-language control of Felix via a local LLM

What shipped on `feature/llm-control` is the **minimal functional slice**: the
`felix_llm` package (teach-by-driving location registry + a 5-tool LLM agent that
drives via `NavigateToPose`). This doc records the full design space so the
deferred survivors are a known backlog, not lost context.

## Grounding Context (Codebase)
- **Actuation:** `/cmd_vel` (Twist, open-loop, 0.5 s watchdog) via `bridge_node`;
  autonomous goals via `/goal_pose` / the `NavigateToPose` action → `bt_navigator`
  (NavFn + MPPI). `felix_llm` uses the **action** (status feedback, no relay).
- **State:** `/amcl_pose` (map frame), TF `map→base_link`. Map `maps/felix_map.yaml`.
  No named locations existed — metric only (now solved by the registry).
- **Obstacles:** `/scan`, `/tof` (always-on mm), Nav2 costmaps own avoidance.
- **LLM:** Nemotron-3-Nano-4B via `llm_server.sh` (llama.cpp, OpenAI-compatible
  `:8080/v1`, `--jinja --tools all`, `<think>` traces, ctx 8192). This work is the
  deferred unit **"U5"** of `docs/plans/2026-06-23-...-nemotron-...-plan.md`.
- **Constraints:** 8 GB unified memory (LLM time-slices CUDA with YOLO — don't run
  both at full tilt); **Nemotron is not in llama.cpp's native tool-call handlers**
  → generic jinja fallback (less reliable); must coexist with default teleop.

## Topic Axes
- A1 Command understanding & dispatch (utterance → tool call; GBNF, latency)
- A2 Skill library & ROS actuation (typed tools wrapping Nav2)
- A3 Spatial grounding (named locations; "where am I")
- A4 Reactive behavior composition ("until obstacle then turn")
- A5 Safety & arbitration (guardrails below LLM reach; teleop coexistence)

## Ranked Ideas

### 1. Single skill registry as the spine — **A2** — *partially shipped*
**Description:** Each capability is a pure function the agent exposes as an OpenAI
tool schema; later the same functions wrap into an MCP server with ~no rework.
**Basis:** `direct:` the "direct now, MCP-optional-later" decision + `external:` llama_ros / ROSA typed-tool registries.
**Rationale:** Answers the MCP-vs-direct question structurally; adding a skill is one function.
**Shipped:** 5 tools in `agent_node.py` (`go_to`, `list_places`, `save_place`, `where_am_i`, `stop`). **Deferred:** auto-generated schema + MCP manifest from one source.
**Confidence:** 88% · **Complexity:** Medium · **Status:** Explored

### 2. GBNF grammar-constrained intent decoding — **A1** — *deferred*
**Description:** Generate a GBNF grammar from the tool registry so Nemotron can only emit valid, in-schema tool calls (or a flat `GOTO <place>` DSL).
**Basis:** `external:` Nemotron isn't natively handled by llama.cpp → unreliable jinja fallback; llama_ros `grammar_schema` is the fix.
**Rationale:** Highest-leverage reliability move; keeps Nemotron + the 8 GB budget instead of swapping models.
**Downsides:** Must confirm GBNF composes with `--jinja --tools all`.
**Confidence:** 85% · **Complexity:** Medium · **Status:** Unexplored (first thing to add if tool-calling proves flaky on hardware)

### 3. Reactive commands = sequenced Nav2 Behavior Server skills — **A4** — *deferred*
**Description:** "Drive forward until obstacle, turn right, proceed when clear" = `DriveOnHeading` (costmap self-terminates) → `Spin(-90°)` → `DriveOnHeading`; LLM only sequences.
**Basis:** `external:` Nav2 Behavior Server built-ins are the exact primitives; RAI: pure-LLM closed-loop is unreliable.
**Rationale:** Keeps a 4B model out of any real-time loop; reuses battle-tested C++ termination. Out of scope for the "go to named room" slice.
**Confidence:** 84% · **Complexity:** Medium · **Status:** Unexplored

### 4. twist_mux safety floor below LLM reach — **A5** — *deferred (not needed yet)*
**Description:** Arbiter under every command path: velocity clamp + always-on `/tof` kinematic envelope + teleop deadman priority. Needed once anything writes raw `/cmd_vel`.
**Basis:** `direct:` the safety gap + always-on `/tof` + `external:` flight-envelope protection / RoboGuard.
**Rationale:** The shipped slice routes ALL motion through Nav2 (costmap-safe), so this is deferred until a reactive/raw-cmd_vel skill exists — at which point it's mandatory.
**Confidence:** 90% · **Complexity:** Medium · **Status:** Unexplored

### 5. Teach-by-driving location registry + landmark "where are you" — **A3** — *shipped*
**Description:** Drive there, save current `/amcl_pose` under a name; "go to X" is a fuzzy lookup → NavigateToPose; reverse-lookup answers "where are you" in landmarks.
**Basis:** `direct:` no named locations today + default teleop workflow + `external:` ROSA LOCATIONS dict, voice-assistant slot-filling.
**Rationale:** Lowest-friction path to the headline commands; no dependency on the unbuilt `semantic_map_node`.
**Shipped:** `LocationStore` + `teach` CLI + `save_place`/`where_am_i` tools.
**Confidence:** 82% · **Complexity:** Low–Medium · **Status:** Explored

### 6. Interpret/commit authority separation + ATC readback — **A5/A1** — *deferred*
**Description:** Stateless LLM "call-taker" emits intent with no actuation authority; a deterministic dispatcher solely owns action clients + send-time checks; content-bearing readback for ambiguous commands.
**Basis:** `external:` emergency-dispatch interpret/commit split + ATC readback; `reasoned:` a hallucination can't become motion without crossing a boundary the LLM can't author.
**Rationale:** Different layer than #4 (software authority vs kinematic floor). The shipped agent already centralizes actuation in one node — a lighter version of this.
**Confidence:** 80% · **Complexity:** Medium · **Status:** Unexplored

### 7. Eval harness keyed to the registry — **A1** — *deferred*
**Description:** A YAML corpus of utterance → expected tool-call, run offline against llama-server in sub-second CI; every new skill auto-contributes a stub.
**Basis:** `external:` you'll churn on model/quant/prompt given Nemotron's tool-calling uncertainty — turns regressions into a CI check, not a hardware test.
**Rationale:** Highest compounding leverage; pairs with #2.
**Confidence:** 83% · **Complexity:** Low–Medium · **Status:** Unexplored

## Rejection Summary

| # | Idea | Reason |
|---|------|--------|
| 1 | Single-shot router + reasoning-budget gating | Folded into #2 / agent loop; best-practice, not novel enough alone |
| 2 | Compile-to-BT offline (LLM-as-programmer) | Heavier/researchy; brainstorm variant of #3 |
| 3 | G-code dry-run plan validation | Fold as a send-time check inside #6's dispatcher |
| 4 | Mode annunciator | UI rider on #4 |
| 5 | Reversible "take me back" skills | Brainstorm variant of #1's skill contract |
| 6 | Conversation/state object ("go back") | Emerges once #1+#6 exist |
| 7 | Action-feedback echo | Absorbed into #3 (NavigateToPose action) |
| 8 | Memory-pressure governor / wake-word gating | Real 8 GB optimization, but later |
| 9 | Perception freshness guard | Niche; depends on perception path |
| 10 | Model swap to Qwen2.5-7B | #2 (GBNF) gets the reliability win while keeping Nemotron; keep as fallback |
| 11 | No-lidar / offline-distill grounding | Degraded-mode, speculative |
| 12 | Auto-harvest registry from perception | Compounding evolution of #5; needs `semantic_map_node` (separate track) |
