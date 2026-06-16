---
date: 2026-06-16
topic: slam-remap-map-quality
focus: "mapping/nav wonky after adding re-map logic; map looks good before save, bad in nav. Should we simplify / remove complexity?"
mode: repo-grounded
---

# Ideation: felix_slam re-map logic & saved-map degradation in Nav2

## Grounding Context (Codebase)

- **Two map artifacts.** slam_toolbox's LIVE `/map` is an occupancy gradient (0–100,
  "looks good"); the SAVED `.pgm` that `map_saver_cli` exports and AMCL/Nav2 load is
  a thresholded grid ("looks bad"). `maps/felix_map.yaml`: `mode: trinary`,
  `occupied_thresh: 0.65`, `free_thresh: 0.25`, `resolution: 0.05`.
- **Trinary export** snaps every mid-probability cell (25–65%) to grey "unknown" →
  thin walls / narrow passages / partial A1 returns vanish in the `.pgm`. `save_map.sh:18`
  calls `map_saver_cli -f "$MAP"` with no `--mode`/`--occ`/`--free`, so it defaults to trinary.
- **Save staleness.** `map_saver` saves the *last published* `/map`; with
  `map_update_interval: 5.0` (`slam_toolbox.yaml:39`) the saved grid can lag the live
  view by up to 5 s. No settle/wait step in `save_map.sh`.
- **Re-map / resume path.** `resume_map.sh` calls the `deserialize_map` service.
  `match_type: 2` (given-pose, validated, ~±20° local search, won't flip) vs
  `match_type: 1` ("dock"/START_AT_FIRST_NODE, wide search, snaps 90° in rectangular
  rooms — documented footgun in the script itself). Resume can produce DOUBLE WALLS:
  two sessions carry independent drift histories Ceres can't reconcile without a
  linking loop closure; given-pose off by >½ cell → globally offset doubled walls.
- **Solver.** `ceres_loss_function: None` (`slam_toolbox.yaml:32`) — no outlier
  rejection. Open-loop mecanum: heading is the least-trustworthy DOF (slip unobserved).
- **External consensus.** Single-session mapping beats resume for any space coverable
  in one drive; resume is a power-user feature for large/battery-limited spaces
  (slam_toolbox README + issues #677, #281).
- **Prior art.** `docs/ideation/2026-06-11-autonomy-stack-ideation.md` idea #5 already
  proposed slam_toolbox localization mode (scan-match the posegraph) to retire AMCL+grid.
- No `docs/solutions/` knowledge base; mapping decisions live only in auto-memory.

## Key reframe

"Wonky" is **two independent problems conflated**:
- **(a)** The saved map *looks* worse than live — this is the **trinary export**, present
  before the re-map work; fixable in one line, removes nothing.
- **(b)** Geometry actually degraded **after adding re-map logic** — this is the **resume
  path** (double walls / 90° dock flip).

So "simplify?": fix (a) with a flag; yes-simplify (b) since it's the actual regression;
treat the deeper localization-mode fork as a deliberate, separate decision.

## Topic Axes

- A. Save-time fidelity (live → `.pgm` export)
- B. Resume / continuation correctness
- C. Localization architecture (AMCL + grid vs slam_toolbox localization mode)
- D. Workflow / operator ergonomics  (folded into idea 3)
- E. Verification & measurement

## Ranked Ideas

### 1. Fix the export: save `scale` mode + lower `free_thresh`
**Description:** Change `save_map.sh`'s `map_saver_cli` call to `--mode scale` (PNG, keeps
the gradient) or at minimum `--free 0.196`, so mid-probability cells survive into the
artifact AMCL/Nav2 loads.
**Axis:** A
**Basis:** `direct:` `maps/felix_map.yaml` `mode: trinary, free_thresh: 0.25`; `save_map.sh:18`
defaults to trinary.
**Rationale:** Most likely cause of "good live / bad saved"; one-line, touches no architecture.
**Downsides:** AMCL/costmap may prefer crisp binary obstacles; verify both. Scale+PGM loses
unknown encoding (use PNG).
**Confidence:** 80% **Complexity:** Low **Status:** Explored

### 2. Flush a fresh render before saving
**Description:** Stop the robot, drop `map_update_interval` / wait one cycle, then save, so the
`.pgm` reflects the live view rather than a stale frame.
**Axis:** A
**Basis:** `direct:` `slam_toolbox.yaml:39 map_update_interval: 5.0`; `save_map.sh` has no settle step.
**Rationale:** Explains "looks good *before* save" if the badness is timing, not thresholds.
**Downsides:** No-op if the operator already pauses before saving.
**Confidence:** 55% **Complexity:** Low **Status:** Unexplored

### 3. Simplify now: single-session mapping, retire/gate the resume path
**Description:** Make single-session the blessed workflow; archive `resume_map.sh` behind a flag.
If resume is kept, delete the `dock` (match_type 1) mode and guard the script to given-pose;
record the tradeoff in a durable doc (axis D).
**Axis:** B
**Basis:** `direct:` `resume_map.sh:36-39` exposes `dock` with its own 90°-flip warning; double
walls are intrinsic to resume. `external:` single-session > resume consensus.
**Rationale:** Directly answers the user's question — the wonkiness arrived *with* the re-map logic.
**Downsides:** Lose incremental extension; re-map whole space each time.
**Confidence:** 75% **Complexity:** Low **Status:** Unexplored

### 4. Turn on a robust Ceres loss (`HuberLoss`)
**Description:** Set `ceres_loss_function: HuberLoss` so a bad scan-match doesn't warp the whole
graph at full weight.
**Axis:** B
**Basis:** `direct:` `slam_toolbox.yaml:32 ceres_loss_function: None`; heading least-trustworthy DOF.
`external:` slam_toolbox exposes HuberLoss for outlier-robust optimization.
**Rationale:** Miscalibrated default for an open-loop chassis; one word helps every map and resume.
**Downsides:** Can under-fit if odom is actually good; verify with idea 6.
**Confidence:** 65% **Complexity:** Low **Status:** Explored

### 5. Architectural fork: slam_toolbox localization mode, retire AMCL + `.pgm`
**Description:** Run `mode: localization` scan-matching against the `.posegraph` already
serialized — no trinary export, no AMCL, no `deserialize_map` dance.
**Axis:** C
**Basis:** `direct:` `slam_toolbox.yaml:19` has the `mode:` switch; `save_map.sh:21` already
serializes `.posegraph/.data`; prior-ideation idea #5.
**Rationale:** Deepest simplification — deletes the lossy-raster + AMCL leg *and* the resume problem.
**Downsides:** Trades AMCL's known failure modes for scan-match-only under slipping odom; symmetric
rooms need a heading hint. Bigger change — decide deliberately, not in the same PR as 1–4.
**Confidence:** 60% **Complexity:** Medium **Status:** Unexplored

### 6. Make "is the map good?" a number
**Description:** Now: a script diffing live `/map` vs reloaded `.pgm` (occupied/free/unknown ratios,
wall-thinness) to tell trinary-erosion from stale-frame from double-wall. Later: a bag-replay
eval harness scoring map RMSE/IoU + final-pose error.
**Axis:** E
**Basis:** `reasoned:` the complaint is purely qualitative; each cause has a distinct measurable
signature. `direct:` repo defers tuning "once verified on hardware" with no metric.
**Rationale:** Tells you *which* fix helped instead of eyeballing; compounds across all future tuning.
**Downsides:** Full bag harness is real effort; start with the cheap histogram check.
**Confidence:** 70% **Complexity:** Low (check) → Medium (harness) **Status:** Explored (check)

## Rejection Summary

| # | Idea | Reason Rejected |
|---|------|-----------------|
| 1 | Save both trinary+scale A/B dump | Subsumed by idea 1 (verification sub-step) |
| 2 | Resume = rebase/seeded-anchor match | Brainstorm detail if resume is kept (idea 3) |
| 3 | Tighten `minimum_travel_*` node spacing | Speculative tuning; gate behind idea 6 measurement |
| 4 | Lifelong / no-saved-map SLAM | Variant of idea 5; loses repeatable maps |
| 5 | Split planning basemap vs localization layer (GIS) | Adds complexity — counter to "simplify"; overlaps idea 5 |
| 6 | Zero-step atomic save on exit | Premature automation; flush folded into idea 2 |
| 7 | HDR-style offline multi-session fusion | Too expensive vs value; large-space problem not yet present |
| - | axis D (pure ergonomics) | Folded into idea 3 (script guard + SSOT doc) rather than standing alone |
