---
date: 2026-06-11
topic: autonomy-stack
focus: autonomy/navigation stack (odometry/EKF, SLAM/AMCL, Nav2/MPPI) running on hardware
mode: repo-grounded
---

# Ideation: Felix Autonomy Stack

Scope: the autonomy/navigation stack currently running on hardware — wheel-encoder
odometry + IMU → robot_localization EKF → slam_toolbox / AMCL → Nav2 (NavFn + MPPI)
→ `/cmd_vel`. ~48 raw candidates across 6 ideation frames, deduped to 7 survivors.

## Codebase Context

Custom 4-wheel **mecanum** robot, Yahboom ROSMASTER board, NVIDIA Jetson Orin, ROS 2
Humble; repo root is the colcon workspace (packages under `src/`). Drive is **open
loop** (per-wheel percent duty via `set_motor()`, no firmware PID); `config.yml` is the
single source of truth for geometry/motor/calibration. EKF (50 Hz) fuses wheel
`vx`/`vy` + IMU **gyro-Z yaw-rate** → `odom→base_link`; accel unfused; gyro-Z
board-negation is SETTLED. slam_toolbox maps / AMCL localizes → `map→odom`. Nav2 =
NavFn + MPPI (holonomic) + behaviors + BT navigator.

Key pain points driving this ideation: (1) open-loop drift, heading least
trustworthy (mecanum slip unobserved by encoders); (2) no closed-loop velocity PID;
(3) IMU accel unfused; (4) MPPI conservative defaults (~44% v_max / ~26% wz_max),
likely critics fighting lateral motion; (5) map resume is a fragile `deserialize_map`
service with a 90°-flip dock failure; (6) no standstill (ZUPT) correction; (7) Foxglove
nav pins are plain topic publishes with footguns; (8) no perception/streaming yet;
(9) no `docs/solutions/` knowledge base — learnings live only in auto-memory.

External best practice (web research): MPPI `motion_model: "Omni"` is required to sample
`vy` (else lateral degrades to heading-first turns); disable PreferForwardCritic, zero
PathAngleCritic, keep GoalAngleCritic; horizon rule `time_steps × model_dt × vx_max <
local_costmap_radius`. slam_toolbox: HuberLoss ceres loss for slipping odometry; localization
mode replaces AMCL. robot_localization: fuse velocities only + IMU vyaw; ZUPT for standstill
drift. Humble add-ons: nav2_collision_monitor, nav2_velocity_smoother, opennav_docking.

## Topic Axes

- A. Drive & odometry — open-loop mecanum, encoder odom, velocity_scale, slip, heading trust
- B. State estimation (EKF) — vx/vy/yaw-rate fusion, IMU gyro-Z, ZUPT, covariance balance
- C. Mapping & localization — slam_toolbox loop closure/resume, AMCL convergence, relocalization
- D. Navigation behavior — Nav2 MPPI critics & speed, NavFn, recoveries, holonomic planning
- E. Operations, safety & observability — collision monitor/smoother/docking, diagnostics, calibration

## Ranked Ideas

### 1. ZUPT — zero-velocity update gated on `cmd_vel ≈ 0`
**Description:** When commanded twist and encoder deltas are ~zero, inject a zero-velocity
pseudo-measurement (vx=vy=vyaw=0, tight covariance) into the EKF so stationary IMU gyro-Z
drift stops accumulating heading error. Gate on both cmd_vel≈0 AND encoder-delta≈0 to also
catch sub-deadband stall (≈14.6% duty).
**Axis:** B
**Basis:** `external:` web research — "ZUPT: zero IMU yaw rate when cmd_vel≈0 — standard fix
for stationary drift"; `direct:` pain point #6. Converged across all 6 ideation frames.
**Rationale:** Heading is the least-trustworthy state; much of its error is bias accrued while
parked between goals — exactly when a free, high-confidence "at rest" measurement is available.
**Downsides:** Must not clamp during commanded-but-coasting; threshold needs a decision;
pre-EKF filter node vs EKF config is an implementation fork.
**Confidence:** 85%
**Complexity:** Low
**Status:** Unexplored

### 2. MPPI Omni motion model + critic rebalance + horizon-bounded speed raise
**Description:** Set MPPI `motion_model: "Omni"` so it samples `vy`, disable PreferForwardCritic,
zero/disable PathAngleCritic, keep GoalAngleCritic, and raise speed limits off the conservative
defaults bounded by `time_steps × model_dt × vx_max < local_costmap_radius` (tune down from the
real hardware envelope rather than up from timid).
**Axis:** D
**Basis:** `external:` without Omni "lateral motion degrades to heading-first turns"; disable
PreferForward/zero PathAngle for omni; horizon constraint. `direct:` pain point #4 (roadmap wants the raise).
**Rationale:** The robot is mechanically holonomic but currently driven like a diff-drive — this
unlocks strafing, the chassis's defining capability, and is the documented root cause of "MPPI fights lateral motion."
**Downsides:** Changes observable behavior; verify Omni and speed-raise separately; needs hardware
validation and a ≥2 m local costmap.
**Confidence:** 80%
**Complexity:** Medium
**Status:** Unexplored

### 3. Collision monitor + velocity smoother safety/smoothing layer below MPPI
**Description:** Insert `nav2_collision_monitor` (holonomic STOP/slowdown polygon off the RPLIDAR,
independent of the BT) and `nav2_velocity_smoother` (bounds jerk, holonomic vy) on the `/cmd_vel`
path between controller and bridge.
**Axis:** E
**Basis:** `external:` both Humble-ready, holonomic add-ons; `direct:` CLAUDE.md safety section (stand
+ Ctrl-C are the only current guards); `reasoned:` abrupt duty steps on a PID-less drivetrain become
wheelspin → odometry error, so smoothing helps estimation too.
**Rationale:** A controller-independent hard floor is the precondition that makes raising MPPI speeds
(#2) safe on a custom, hard-to-repair chassis — it decouples "go faster" from "stay safe."
**Downsides:** Adds nodes with halt authority on the critical path; pipeline order and polygon geometry
need sign-off; scope (does it catch teleop too?) is a decision.
**Confidence:** 80%
**Complexity:** Low–Medium
**Status:** Unexplored

### 4. Slip-as-observable: 4-wheel kinematic residual → EKF covariance
**Description:** Mecanum has 4 encoders but 3 body DOF — `wheels_to_body` is over-determined, so the
least-squares residual is a real-time slip signal. Publish it as a diagnostic and use it to inflate
wheel-odometry covariance into the EKF when slip actually happens, instead of a permanent static distrust.
**Axis:** A
**Basis:** `reasoned:` 4 constraints → 3-DOF twist means the discarded 4th equation's residual is
nonzero exactly under slip/scrub (automotive ESC solves the identical "more wheel sensors than DOF"
problem this way); reframes "slip is unobserved" as "unobserved only if you discard the 4th equation."
**Rationale:** Attacks pain point #1 at the root — turns the most-distrusted quantity into something the
EKF can down-weight conditionally, no new hardware. Signal also feeds #7.
**Downsides:** Residual sensitivity to calibration error; confirm it's computable in the bridge poll
timer; per-wheel covariance vs cruder global "trust IMU during slip" knob is a real fork.
**Confidence:** 65%
**Complexity:** Medium
**Status:** Unexplored

### 5. slam_toolbox localization mode, replacing the AMCL + `match_type` resume ritual
**Description:** Use slam_toolbox's `localization` mode (continuous scan-matching against the saved
posegraph) instead of `nav2_map_server`+AMCL. Eliminates AMCL initialpose fragility, the cosmetic
extrapolation warning, and the `deserialize_map`/`match_type` 90°-flip dock dance, while keeping
lifelong map extension open. Pair with HuberLoss ceres loss for slipping odometry.
**Axis:** C
**Basis:** `external:` slam_toolbox localization mode + HuberLoss rec; `direct:` pain points #5 and #7.
**Rationale:** Collapses mapper+AMCL into one scan-matcher and removes a class of init bugs — high-value
simplification for a stack where relocalization is a recurring pain.
**Downsides:** Trades AMCL's well-understood particle-filter failure modes for scan-match-only behavior
under slipping odometry; symmetric-room ambiguity still needs a heading hint; non-trivial rewire of `felix_slam`.
**Confidence:** 60%
**Complexity:** Medium
**Status:** Unexplored

### 6. Bag-based replay/eval harness for EKF/SLAM/MPPI tuning
**Description:** Record canonical `rosbag2` runs (straight drive, spin-in-place, lateral strafe,
loop-closure loop) with ground-truth checkpoints, plus a script that replays a bag against a given config
and scores it (final-pose error, heading drift, map RMSE, MPPI path deviation). Tuning becomes "change a
param → rerun → compare a number."
**Axis:** B (cross-cutting leverage)
**Basis:** `reasoned:` EKF/SLAM/MPPI all have many free params and the codebase repeatedly defers tuning
"once verified on hardware" with no measurement method; mirrors the `ce-optimize` metric-driven loop.
**Rationale:** Highest-compounding asset here — makes survivors #1/#2/#4/#5 measurable experiments instead
of floor-eyeballing, and catches regressions. Bags double as the diagnostics/learnings corpus.
**Downsides:** Upfront build cost; ground-truth on a drifting open-loop robot is itself hard (tape-measure
protocol or fiducials); replay determinism caveats.
**Confidence:** 75%
**Complexity:** Medium–High
**Status:** Unexplored

### 7. `/diagnostics` golden-signals health layer + on-breach incident bag recorder
**Description:** A `felix_diag` layer (`diagnostic_updater`/`aggregator`) publishing a few SRE-style golden
signals — serial-link alive, encoder/IMU staleness, `map→odom` TF freshness, EKF innovation magnitude,
AMCL/scan-match divergence, MPPI loop time, cmd_vel saturation — into one Foxglove panel, with a rosbag
ring-buffer that auto-snapshots on an SLO breach so field failures are replayable, not just logged.
**Axis:** E
**Basis:** `external:` Google SRE golden signals + ROS `diagnostic_updater`/`rosbag2` snapshot; `direct:`
axis E names diagnostics, and the TF/topic chain has many silent failure points with no unified health signal.
**Rationale:** You can't tune what you can't observe — the force-multiplier under #2/#4/#5/#6; the divergence
signal gives early kidnap/localization-loss warning before Nav2 drives into a wall.
**Downsides:** Pick the real 4–5 signals (not 20); continuous ring-buffer disk/IO cost on the Orin;
overlaps slightly with #6's instrumentation.
**Confidence:** 70%
**Complexity:** Medium
**Status:** Unexplored

## Sequencing notes

- **#3 → #2** (safety layer before raising speeds) and **#1 / #4** (cheap estimation wins) are the
  low-risk first moves.
- **#6** is the multiplier that turns #1/#2/#4/#5 from "tweak and hope" into measured changes — worth
  building early if a tuning campaign is imminent.
- **#5** is the one architectural fork (drop AMCL); weigh against slip-tolerance before committing.

## Rejection Summary

| # | Idea | Reason Rejected |
|---|------|-----------------|
| 1 | Closed-loop per-wheel velocity PID in bridge | High-burden architectural shift; cheaper open-loop fixes (#4, #1) land first |
| 2 | Auto/online + per-axis `velocity_scale` write-back | Folded into #4 / calibration workflow |
| 3 | Accel as low-weight slip cross-check; F1/ESC cmd-scheduled covariance | Same target as #4 via cruder proxies; #4 preferred |
| 4 | docs/solutions KB seeded from learnings | Valuable but meta; capture via `/ce-compound` as tuning proceeds |
| 5 | Dock-anchored resume (opennav_docking) | Strong, but needs a physical dock — revisit after #5 |
| 6 | WAL/atomic map serialization | Real durability win, secondary to #5 |
| 7 | Disposable-vs-lifelong map | Strategic fork — better as a brainstorm question |
| 8 | Holonomic global planner (NavFn→Smac omni) | Good assumption-break; secondary to #2, revisit if paths stay car-like |
| 9 | Context-steering lateral recovery | Novel but exotic; after #2+#3 |
| 10 | Aviation heading-voting / pre-flight readiness gate | Folded into #7 as signals/checks |
| 11 | Foxglove committed layout JSON | Small DX win; bundle opportunistically |
| 12 | No-LIDAR degraded mode; auto-relocalization sweep | Resilience nice-to-haves; subsumed by #3/#5 |
