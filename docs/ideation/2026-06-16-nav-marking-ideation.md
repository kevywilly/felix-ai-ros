---
date: 2026-06-16
topic: nav-marking
focus: marking detected items on the map and using them in navigation (the layer above 3D grounding)
mode: repo-grounded
---

# Ideation: Marking Items on the Map for Navigation

Continues the perception discussion from `2026-06-16-yolo-perception-ideation.md`. Since that
session the `felix_perception` foundation shipped: idea #1 there (raw path + REP-105 optical
frame + camera calibration — now done) and idea #2 (bearing-wedge YOLO∩RPLIDAR 3D grounding →
`/perception/objects`) are implemented. This session scopes the **next layer**: getting those
map-frame detections *out of the visualization layer and into navigation*.

## Grounding Context (Codebase)

Custom 4-wheel **holonomic mecanum** robot, Yahboom ROSMASTER, NVIDIA Jetson Orin, ROS 2 Humble.
`felix_perception`: `detector_node` (YOLO→TensorRT FP16 → `/perception/detections`,
`vision_msgs/Detection2DArray`, **no track IDs**); `fusion_node` (bearing-wedge: YOLO bbox
horizontal extent → bearing wedge in `camera_optical_link` → select RPLIDAR returns whose bearing
lands in the wedge → nearest cluster median range → object position in the `map` frame).

**The gap this session is about:** `/perception/objects` is a `MarkerArray` that is **viz-only and
ephemeral** — 1.0 s marker lifetime, `DELETEALL` every cycle, **no persistence, no object IDs, no
cross-frame association/dedup**. **Nothing in Nav2 consumes it** — detections die at the
RViz/Foxglove layer.

Nav2 (`felix_nav/config/nav2_params.yaml`): NavFn planner + **MPPI "Omni"** holonomic controller.
Local (rolling 3×3) + global costmaps consume **only `/scan`** via `ObstacleLayer`; layers are
`static_layer`, `obstacle_layer`, `inflation_layer` — **no keepout/filter/semantic layer**. Goals
arrive as a single `/goal_pose` `PoseStamped` (`bt_navigator` NavigateToPose); no semantic "go to
object X". AMCL localizes on the saved map (`maps/felix_map.yaml`). **Mono** forward CSI camera (no
depth); RPLIDAR is a single horizontal plane (~0.115 m up) → only floor-crossing objects get a
range; off-plane objects and the unlocalized case publish **no** marker.

**External grounding (web research):** STVL (`spatio_temporal_voxel_layer`) accepts `PointCloud2`
as a Nav2 costmap obstacle source with time decay; Nav2 **Keepout Filter** + `VectorObjectServer`
rasterize YAML polygons/circles into a filter mask; Nav2 **Route Server** (`ros-humble-nav2-route`)
does graph nav with semantic edge/node penalties; Nav2 **Dynamic Object Following** uses a
`GoalUpdater` BT node to chase a live-updating `PoseStamped`; `ros_iou_tracking` / Ultralytics
`track persist=True` give stable track IDs; the **April-2026 Sensors "Angular Sector Fusion"
paper** is an exact hardware match (mono + 2D lidar + Jetson) persisting objects to a GeoJSON
semantic map feeding the Nav2 Route Server; classical **Elfes log-odds** occupancy accumulation is
the analogy for confidence-decay per map cell; the realistic DIY starting point is a node holding
`dict[(class, grid_cell)] → {count, last_seen, pose}` publishing persistent markers only at
`count ≥ N`.

## Topic Axes (navigation surface)

- N1. Representation & persistence — ephemeral MarkerArray → tracked/IDed/decaying semantic object store
- N2. Influence on planning/costmaps — object-aware costmap layers, keepout/avoid, dynamic obstacles
- N3. Commanding by semantics — "go to the chair / find a person": object-as-goal, semantic waypoints, HA/voice
- N4. Spatial-temporal reasoning — moved/appeared/gone, static vs dynamic, last-seen, room/zone anchoring
- N5. Trust & failure handling — bearing-wedge single-plane accuracy, gating bad placements, teach/correct

## Ranked Ideas

### 1. Probabilistic persistent semantic object store (the keystone)
**Description:** A `semantic_map_node` between `fusion_node` and every consumer. Holds objects keyed
by `(class, map-grid-cell)` — or track ID, see fork — with **Bayesian log-odds confidence**,
`last_seen`, pose, and covariance. Each detection adds evidence; each in-FOV non-detection subtracts
it. Promotes a candidate to a committed object at a confidence threshold and republishes the
committed set as a **latched `vision_msgs/Detection3DArray` + a query service** — the single contract
every consumer (costmap, goals, HA, dataset) reads.
**Axis:** N1
**Basis:** `direct:` `/perception/objects` is viz-only/ephemeral (1.0 s lifetime, DELETEALL),
nothing in Nav2 consumes it; `external:` Elfes log-odds occupancy accumulation + the DIY
`dict[(class,cell)]→{count,last_seen,pose}` pattern; standard `vision_msgs` typing gives off-the-shelf
rosbag/Nav2/vision tooling interop for free.
**Rationale:** The one missing primitive that gates *every* navigation use. Build it once and
costmap/commanding/HA/docking become thin adapters instead of parallel rewrites. Log-odds makes
persistence, decay, and trust **one number** instead of three bolt-on heuristics.
**Key design fork:** association in **grid-space** (no tracker — cheap, but double-counts under AMCL
jitter) vs **ID-space** (`Ultralytics persist=True` / `ros_iou_tracking` — survives pose jumps and
can represent "the same chair moved", but adds a tracker dependency on the Orin).
**Downsides:** Schema is load-bearing — a wrong dedup key is inherited everywhere. Log-odds/covariance
tuning.
**Confidence:** 92% **Complexity:** Medium **Status:** Unexplored

### 2. Object-aware costmap layer (confirmed objects → obstacles / keepout)
**Description:** Feed committed objects (idea 1) into Nav2 as real planning cost via two concrete
mechanisms: (a) publish them as a synthetic `PointCloud2` and add an **STVL** observation source to
the local costmap so they mark *and* time-decay; (b) for "avoid" classes (dog, cable, shoe),
auto-rasterize an inflated circle into a **Keepout Filter** mask from a one-line `class → avoid`
policy table — no hand-drawn YAML polygons.
**Axis:** N2
**Basis:** `external:` STVL accepts `PointCloud2` with decay; Keepout Filter + `VectorObjectServer`
rasterize polygons/circles; `direct:` costmaps currently consume only `/scan` via `ObstacleLayer` —
a second observation source slots in cleanly.
**Rationale:** The most direct "make navigation use the detections" path, and it specifically
rescues **off-plane / occluded** objects the single-plane lidar can't defend against (a table edge,
a hanging cable).
**Downsides:** STVL has a known Humble-22.04 slowness (jemalloc `LD_PRELOAD` fix). A wrong placement
becomes a phantom wall — depends hard on idea 7's gating.
**Confidence:** 84% **Complexity:** Medium **Status:** Unexplored

### 3. Influence-map soft cost as an MPPI critic
**Description:** Instead of (or alongside) binary obstacle cells, give each committed object a smooth
potential field — repulsive for "person", attractive for a goal object — summed and sampled by MPPI
as an **extra critic cost**. Game-AI influence maps ported directly onto the controller already in
use.
**Axis:** N2
**Basis:** `reasoned:` MPPI is already a sampling controller summing weighted critic costs over
rollouts (`critics: [...]` in `nav2_params.yaml`) — structurally identical to sampling an influence
field; `direct:` `motion_model: "Omni"` means the robot can *slide* around a person rather than
hard-stop.
**Rationale:** Lets a detected person **bend** a holonomic trajectory smoothly rather than build a
wall the planner routes around — a behavior binary lethal cells can't express, and uniquely natural
for a strafing chassis.
**Downsides:** Requires a custom MPPI critic plugin (C++), more than a config change. Weight tuning
against the existing critics.
**Confidence:** 70% **Complexity:** High **Status:** Unexplored

### 4. Semantic "go to the X" goals + live GoalUpdater chase
**Description:** Replace clicking an XY pose with a `/goto_named` (or "go to nearest `<class>`")
action: look up the highest-confidence instance in the store, synthesize a standoff approach pose,
call `NavigateToPose`. For moving/uncertain targets, wire it through Nav2's **GoalUpdater** BT node
so the goal slews as the object's stored pose refines.
**Axis:** N3
**Basis:** `direct:` goals are a single `/goal_pose` PoseStamped today, no semantic lookup;
`external:` Nav2 Dynamic Object Following uses GoalUpdater to chase a live PoseStamped; the ASF paper
queries its semantic map for navigation.
**Rationale:** The visible payoff that makes the whole store *useful* — "go to the charger / find a
person" — and the natural binding point for the roadmap's Home Assistant/voice triggers.
**Downsides:** Standoff-pose computation (which side, how far, facing) is fiddly; ambiguity when
multiple instances of a class exist.
**Confidence:** 82% **Complexity:** Medium **Status:** Unexplored

### 5. Per-class temporal decay (half-life / NOTAM-style TTL)
**Description:** Each object carries a **class-specific persistence prior** — a wall/furniture decays
over hours, a person over seconds. Costmap/goal influence = current confidence × class cost, so
stale marks fade automatically (aviation NOTAM expiry / STVL decay, but per-class instead of one
global timeout).
**Axis:** N4
**Basis:** `external:` STVL per-source decay + NOTAM per-item validity windows; `direct:` the current
MarkerArray uses a flat 1.0 s lifetime — no real temporal model.
**Rationale:** A patrol robot re-visits the same space; a class-aware half-life is the difference
between trusting a clear path and routing around the ghost of a person who left 30 s ago.
**Downsides:** Per-class half-life table to author and tune; mis-set rates either keep ghosts or
forget real furniture.
**Confidence:** 80% **Complexity:** Low **Status:** Unexplored

### 6. Bearing-only landmark triangulation (recover the un-ranged majority)
**Description:** Today a detection with no lidar return in its wedge is **silently dropped**
(off-plane object, or unlocalized). Instead, store it as a **bearing-only landmark** (ray + class,
range unknown); when the robot re-sees it from a different pose, **triangulate** the intersection
into a placed object.
**Axis:** N4
**Basis:** `direct:` `fusion_node` only places floor-crossing objects; off-plane/unlocalized produce
no marker; `reasoned:` two bearings from distinct poses intersect to a point without depth —
recovers data the single plane structurally cannot.
**Rationale:** On a mono + single-plane robot, most detections (anything not at ~0.115 m height) are
thrown away. This converts the hardware's biggest blind spot into usable map landmarks instead of
nothing.
**Downsides:** Needs robust cross-view data association (harder without track IDs); triangulation
degrades with poor localization.
**Confidence:** 71% **Complexity:** High **Status:** Unexplored

### 7. Trust gate: multi-view confirmation + operator teach-correct
**Description:** Nothing influences navigation until it has earned trust two ways: (a) **multi-view
confirmation** — an object promotes only after ≥2 sightings from distinct poses with agreeing ranges
(SIR-style Tentative→Confirmed→Recovering states), so single-frame YOLO flickers never reach the
planner; and (b) a **Foxglove teach/correct channel** — operator clicks the true location or a
"delete this object" pin to fix or remove a bad entry, reusing the existing pin-as-topic-publish
pattern.
**Axis:** N5
**Basis:** `direct:` mono + single-plane + no track IDs makes single-shot fusion error-prone (the
node's own docstring flags this); Foxglove nav pins are already plain topic publishes (CLAUDE.md);
`external:` epidemiology SIR threshold/recovery model for converting noisy per-sample signals into a
trusted state.
**Rationale:** Persistence **amplifies** errors — a wrong placement becomes a permanent phantom wall.
Gating + a one-click human override is what makes the persistent store safe to wire into the planner
rather than a liability.
**Downsides:** Confirmation adds latency before an object is usable; correction UX to build; conflict
resolution when operator and perception disagree.
**Confidence:** 83% **Complexity:** Medium **Status:** Unexplored

**Build order:** #1 (store) → #7 (gate) → #2 (costmap) / #4 (goals) in parallel → #5 (decay) →
#6 (triangulation) / #3 (influence-map) as advanced follow-ups. #1 is the unavoidable first brick;
#7 should land with or right after it so the store never feeds the planner garbage.

## Rejection Summary

| # | Idea | Reason Rejected |
|---|------|-----------------|
| 1 | Detection3DArray + stable-ID contract; log-odds law | Merged into #1 — facets of the keystone store, not separate features |
| 2 | ID-free grid clustering vs track IDs | Folded into #1 as the explicit association design fork |
| 3 | Sonar contact w/ covariance→inflation; cartographic point→polygon generalization | Folded into #1 (covariance field) / #2 (keepout polygons) |
| 4 | Auto-keepout from class avoid-list | Merged into #2 as mechanism (b) |
| 5 | Per-class policy table (ATC separation minima) | Cross-cuts #2/#4/#5; captured as the shared `class → behavior` table, not its own idea |
| 6 | Patrol-the-unknowns (staleness-driven autonomous revisit) | Scope overrun — autonomous exploration/coverage is a separate topic from marking-for-nav |
| 7 | Cross-run density/pheromone heatmap prior | Longer-horizon learned prior; better as a brainstorm variant once the store exists |
| 8 | Mission-planner altitude (BT goal sequencing) | Overlaps #4; the sequencing layer is a brainstorm concern once `/goto_named` exists |
| 9 | MVCC plan-stable snapshots | Real concurrency concern but an implementation detail of #1/#2, not a product idea |
| 10 | Detection provenance log = dataset flywheel | High-leverage but belongs to the dataset/perception-improvement topic (prior doc), not navigation-use |
| 11 | "Perception validates human-authored keepout" (inverted source-of-truth) | Below ambition floor vs #1+#7; a config-time fallback, not a direction |
| 12 | Disagreement-as-signal (lidar∩camera conflict) | A safety/unknown-obstacle topic distinct from marking *named* items |

Axis spread across survivors: N1×1, N2×2, N3×1, N4×2, N5×1 — all five axes covered.

## Deferred / linked

- The roadmap's **Home Assistant/MQTT output**, **dataset flywheel**, and **fiducial docking** all
  bind to idea #1's contract — see `2026-06-11-autonomy-stack-ideation.md` and
  `2026-06-16-yolo-perception-ideation.md`. The fiducial landmark network's relocalization value
  would compound with idea #6 (better localization sharpens triangulation).
- Suggested next: `ce-brainstorm` on idea #1 (the store) — it gates everything else.
