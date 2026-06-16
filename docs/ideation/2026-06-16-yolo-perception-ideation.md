---
date: 2026-06-16
topic: yolo-perception
focus: home-environment perception using YOLO (docking deferred); first build the felix_perception foundation
mode: repo-grounded
---

# Ideation: Felix YOLO Perception (perception-first; docking deferred)

Scope narrowed from a broader "home perception + fiducial front-in docking" session: the
user chose to **start with YOLO perception, no docking yet**. The fiducial/docking ideas
(tag network, strafe visual-servo docking, wall tag bundle, self-describing docks) are
parked for a later session — see the Deferred section. Chosen seed for `ce-brainstorm`:
idea #1 (the `felix_perception` foundation).

## Grounding Context (Codebase)

Custom 4-wheel **holonomic mecanum** robot, Yahboom ROSMASTER board, NVIDIA Jetson Orin,
ROS 2 Humble; repo root is the colcon workspace (packages under `src/`). The camera is the
only perception input today: `src/felix_camera/camera_node.py` publishes
**`/camera/image_raw/compressed`** — lossy JPEG, **640×360 @ 20 fps, quality 60**,
deliberately tuned for low-lag FPV over WiFi (overridable to raw / higher res / 59 fps via
launch args `output_width`/`output_height`/`max_fps`/`jpeg_quality`). Camera is a forward-
facing CSI IMX219 (mounted inverted, flip 180°), frame `camera_link` 0.1335 m forward of
`base_link` at axle height. RPLIDAR present on `/dev/rplidar`. AMCL localizes on a saved
map (`maps/felix_map.yaml`); Nav2 = NavFn + MPPI ("Omni" holonomic).

**Gap:** NO `felix_perception` package yet (planned: YOLO→vision_msgs). NO raw image topic,
NO REP-105 optical frame (URDF comment: "optical frame gets added when felix_perception
lands"), NO camera calibration (`camera_info` K+D), NO `docs/solutions/` knowledge base.
Jetson Orin has spare GPU.

**External grounding:** YOLOv8n/v11n reach ~47–65 FPS on Orin via TensorRT FP16/INT8
(`model.export(format='engine', half=True)`); `l2d_detection` projects RPLIDAR into YOLO
bboxes → 3D `PoseArray`; `mqtt_client` (ika-rwth-aachen) bridges Home Assistant ↔ ROS 2;
domain shift COCO→a specific home is the dominant accuracy gap (on-robot bag capture is
nearly free once tooled).

## Topic Axes (perception surface)

- C1. Camera input & framing — raw vs compressed, resolution, REP-105 optical frame, calibration
- C2. YOLO inference — model choice, TensorRT engine, rate, `vision_msgs` contract
- C3. 2D→3D grounding — lidar∩bbox fusion, projection into the `map` frame
- C4. What detections are for — Home Assistant/MQTT output, semantic map / planogram, nav triggers
- C5. Improvement loop & observability — dataset flywheel, FPV overlay, health gate, eval

## Ranked Ideas

### 1. Stand up `felix_perception` on a raw path with a proper optical frame
**Description:** New `felix_perception` package. Add a perception launch profile publishing
raw (or high-quality) higher-res images on their own topic, keeping the compressed stream
solely for Foxglove FPV. Add `camera_optical_link` (z-forward, REP-105) to the URDF and do
a one-time checkerboard calibration so `camera_info` K+D is real. Node: Ultralytics →
TensorRT FP16 engine → `vision_msgs/Detection2DArray` with correct stamped optical frame.
**Axis:** C1+C2
**Basis:** `direct:` default feed is q60 JPEG @640×360 "for FPV", URDF "optical frame gets
added when felix_perception lands"; `external:` YOLOv8n/v11n 47–65 FPS via TensorRT FP16 on Orin.
**Rationale:** Every later perception capability (3D grounding, fiducials, fusion) inherits
accuracy from the frame + calibration; do it right once so downstream "just works" instead
of each consumer reinventing fragile transforms.
**Downsides:** Thermal/throughput tuning; raw vs high-q-JPEG bandwidth decision; calibration
is a manual chore.
**Confidence:** 88% **Complexity:** Medium **Status:** Explored

### 2. Ground detections in 3D by intersecting YOLO boxes with the RPLIDAR scan
**Description:** A mono camera can't range; project each bbox ray into the RPLIDAR scan
(l2d-style) to get object positions in the `map` frame ("person 2.3 m ahead by the couch")
without a depth camera.
**Axis:** C3
**Basis:** `external:` `l2d_detection` lidar∩YOLO bbox → 3D `PoseArray`; `direct:` robot has
both a forward camera and RPLIDAR.
**Rationale:** Cheapest path to 3D semantics on this exact hardware; turns a screen box into
something navigation and the home model can consume.
**Downsides:** Lidar∩camera time-sync is fiddly; lidar is a 2D plane (height-limited).
**Confidence:** 80% **Complexity:** Medium **Status:** Unexplored

### 3. Decide what detections are for — robot-as-home-sensor via Home Assistant/MQTT
**Description:** Publish detections ("stove on, door open, person in kitchen") into Home
Assistant over one `mqtt_client` bridge; optionally annotate map regions (a "planogram") so
a raw box becomes meaningful (person in bed at 2am = expected; in the locked garage = alert).
HA events can flow back as nav triggers.
**Axis:** C4
**Basis:** `external:` `mqtt_client` ↔ Home Assistant, mmWave occupancy → nav; retail
planogram = spatial-expectation map → anomaly.
**Rationale:** Perception with no consumer is wasted compute; deciding the data contract now
prevents building a detector nobody reads. The real answer to "sensors around the house" —
a roaming robot covers rooms a fixed mesh can't.
**Downsides:** HA dependency; anomaly rules need authoring; roaming-camera privacy.
**Confidence:** 74% **Complexity:** Medium **Status:** Unexplored

### 4. Observability + health from day one: overlay boxes on FPV, gate on a live camera
**Description:** Draw detections + confidence onto the Foxglove video so you can see what the
model sees while tuning on hardware; add a lightweight camera-health check (feed arriving,
recent, non-black, in focus) so a dead/occluded camera fails loudly, not silently.
**Axis:** C5
**Basis:** `external:` AR-HUD overlay legibility pattern; `direct:` single forward camera =
single silent failure point.
**Rationale:** Debugging perception blind on a robot is brutal; makes every later perception
decision debuggable and pays for itself in week one.
**Downsides:** Tooling effort that doesn't ship a "feature" directly.
**Confidence:** 78% **Complexity:** Low **Status:** Unexplored

### 5. Record-while-driving → fine-tune for this home (the dataset flywheel)
**Description:** A "record a bag while I drive every room" op plus an offline auto-label pass
to seed a house-specific dataset; on-robot detections feed back as new labeled data.
**Axis:** C5
**Basis:** `external:` COCO→specific-home domain shift is the dominant accuracy gap; on-robot
bag capture is nearly free once tooled.
**Rationale:** The durable asset is the dataset, not a frozen weight — it compounds with every
drive and doubles as a perception regression set. Sequence after #1–#2 (need the pipeline to
record through it).
**Downsides:** Auto-labeling quality; storage; retrain workflow to maintain.
**Confidence:** 76% **Complexity:** Medium **Status:** Unexplored

**Build order:** #1 → #2 → #4 (parallel) → #3 → #5. Idea #1 is the unavoidable first brick.

## Deferred (parked with docking, for a later session)

From the broader run (`raw-candidates` run a1f3c7d9): fiducial landmark **network**
(dock + AMCL relocalization + ground-truth, teach-by-driving survey); holonomic **strafe
visual-servo** front-in docking with commit-gate + wave-off; multi-tag **wall bundle** as
camera-calibration target; **self-describing docks** (tag payload manifest, resolves
front-in vs back-in fork). Note: the fiducial network's relocalization value is
YOLO-independent and would compound with this perception work — revisit after the foundation.

## Rejection Summary

| # | Idea | Reason Rejected |
|---|------|-----------------|
| 1 | Always-on multi-model scene-graph daemon | Too expensive now; thermal/power risk on Orin; premature |
| 2 | MCU-class perception/docking loop | Not grounded — no MCU in the loop; Orin has spare GPU |
| 3 | Zero-marker YOLO dock (recognize dock itself) | Docking-scoped; deferred. Monocular scale unreliable; brainstorm variant later |
| 4 | Event-gated / on-demand YOLO | Folds into #3 (HA triggers decide when to run) |
| 5 | Perception health gate as standalone | Below ambition floor; folded into #4 |
| - | All fiducial/docking ideas | Out of perception-first scope by user choice; see Deferred |
