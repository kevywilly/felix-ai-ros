# felix-ai-ros

Base driver, teleop, and an autonomy stack (odometry + IMU → EKF → SLAM →
localization → **Nav2**) for a **custom 4-wheel mecanum robot** driven by a Yahboom
ROSMASTER board on an NVIDIA Jetson Orin (ROS 2 Humble).

Because the chassis is custom (not a stock Yahboom frame), the board firmware's
built-in kinematics (`set_car_motion`) do not apply. Instead this project
computes mecanum kinematics from the robot's real geometry in `config.yml` and
drives the four motors directly, open-loop, via `set_motor()`. It also publishes
wheel-encoder **odometry** + `odom→base_link` TF and the board **IMU**.

## Workspace

The repo root is a colcon workspace; packages live under `src/`:

| Package | Role |
|---------|------|
| `felix_base` | Base driver: hardware bridge (`/cmd_vel`→motors), odometry + IMU, ToF array, keyboard teleop, mecanum kinematics, and `config/config.yml`. |
| `felix_localization` | `robot_localization` EKF fusing `/odom` + `/imu/data` → `odom→base_link` (`config/ekf.yaml`). |
| `felix_description` | URDF/TF: publishes `base_footprint→base_link→{imu_link, laser, camera_link}` on `/tf_static` (the EKF, SLAM, and Nav2 consume it). |
| `felix_slam` | RPLIDAR driver + **mapping** (`slam_toolbox` → live map) **and localization** (`nav2_map_server` + **AMCL** on a saved map → `map→odom`). |
| `felix_nav` | **Autonomous navigation**: the Nav2 servers (NavFn planner, **MPPI** holonomic controller, recovery behaviors, BT navigator) → `/cmd_vel`. |
| `felix_camera` | CSI camera → compressed image (FPV in Foxglove). |
| `felix_perception` | **YOLO** detector (Ultralytics on a TensorRT engine) → `/perception/detections` + annotated FPV image, and **lidar-fused** map-frame object markers on `/perception/objects` (`perception:=true`). |
| `felix_llm` | **Natural-language control**: a local LLM (Nemotron via llama.cpp) turns "go to the office", "what do you see", "find the cat", "patrol", "roam" into Nav2 goals + behaviors. Shared skills behind a `/llm/command` agent and an MCP server for the llama-server web UI. |
| `felix_bringup` | Top-level launch composition + params. Three one-shot stacks: `mapping.launch.py` (build a map), `localization.launch.py` (drive on a saved map), `navigation.launch.py` (autonomous + perception/LLM). `felix.launch.py` is the base stack alone. |

Inside `felix_base`:

| Module | Role |
|--------|------|
| `bridge_node.py` | Owns the serial link. Subscribes `/cmd_vel` → `set_motor()`; publishes `/odom` + TF and `/imu/data`. |
| `teleop.py` | Keyboard teleop node. Reads keystrokes, publishes `geometry_msgs/Twist` on `/cmd_vel`. |
| `kinematics.py` | Mecanum inverse + forward kinematics, velocity limits, wheel↔percent mapping. Single source of truth, loads `config.yml`. |
| `odometry.py` | Integrates encoder deltas into a planar pose (dead reckoning). |
| `tof_array_node.py` | Reads the ToF sensor array over serial, publishes `tof`. |
| `rosmaster.py` | Yahboom hardware driver (serial protocol). Vendor code — don't edit. |
| `calibrate.py` | Calibration & verification CLI (see below). |
| `config/config.yml` | All robot geometry, motor specs, and calibration values. **The only place to edit them.** |

## Topics & TF

```
teleop / Nav2 ──/cmd_vel (Twist)──► bridge_node ──► set_motor() ──► motors
                              bridge_node ──/odom (Odometry)──►   + TF odom→base_link
                              bridge_node ──/imu/data (Imu)──►
                              tof_array_node ──/tof (Int16MultiArray)──►
```

Full TF chain when localizing/navigating on a map:

```
map ──► odom ──► base_link ──► {imu_link, laser, camera_link ──► camera_optical_link}
 │        │          │
 │        │          └─ felix_localization EKF  (fuses /odom vx,vy + /imu gyro-Z)
 │        └──────────── (the EKF's odom frame; allowed to drift)
 └─ slam_toolbox (mapping) OR AMCL (localization)  — the drift correction
```

When the EKF is on, the bridge's own `odom→base_link` TF is suppressed so the two
don't fight. `base_link→{imu_link, laser, camera_link}` come from the URDF;
`camera_link→camera_optical_link` is the REP-105 optical frame (z-forward) perception
stamps detections in.

With `perception:=true` (see *Run — perception*), `felix_perception` adds:

```
camera/image_raw/compressed ──► detector ──/perception/detections (Detection2DArray)──►
                                         └──/perception/annotated/compressed (FPV boxes)
                                         └──/camera/camera_info + /perception/status
detector + /scan ──► fusion ──/perception/objects (MarkerArray, map frame)──►
```

## Requirements

- ROS 2 Humble (`source /opt/ros/humble/setup.bash`)
- Python 3 with `rclpy`, `pyserial`, `pyyaml`
- `setuptools<80` for `--symlink-install` (see `BUILD_NOTES.md`)
- `ros-humble-robot-localization` for the EKF (`apt install`; in the Dockerfile)
- The ROSMASTER board on a serial port (default `/dev/myserial`)

## Container (run from the Jetson host)

The workspace lives inside the `felix-ai` Docker container (`docker/Dockerfile`).
Start it foreground (drops you straight into a ROS-sourced shell):

```bash
./scripts/start-container.sh          # docker run -it --rm … (foreground)
```

Or run it **detached** and `exec` in on demand — handy for leaving the robot up
in the background and opening shells as needed:

```bash
DETACH=1 ./docker/run-interactive.sh felix-ai-ros:latest   # background (-dit)
# or the convenience wrapper:
./scripts/start-container-detached.sh

docker exec -it felix-ai bash         # enter; shell is ROS-sourced, cd /felix-ai-ros
docker stop felix-ai                  # stop + remove (container is --rm)
```

`DETACH=1` just swaps `-it` for `-dit` on the same `docker run`; the allocated TTY
keeps the container's default bash alive in the background. Running `ros2 launch …`
inside an `exec` session still stops cleanly on one `Ctrl-C` while the container
itself stays up for the next `exec`.

## Build

Inside the container:

```bash
cd /felix-ai-ros
colcon build --symlink-install
source install/setup.bash
```

## Run — full stack (mapping & driving)

The everyday workflow. One launch brings up **base + EKF + description + SLAM +
camera + Foxglove bridge** as children of a single `ros2 launch`, so **one
`Ctrl-C` cleanly stops all of it** (no orphaned nodes). Drive from the Foxglove
**Teleop** panel and watch the map + FPV in the same UI — no teleop terminal
needed.

```bash
ros2 launch felix_bringup mapping.launch.py     # everything; drive + map + FPV in Foxglove
```

Everything defaults **on**; drop any piece with its `:=false` toggle without
losing the rest:

```bash
ros2 launch felix_bringup mapping.launch.py camera:=false           # skip camera
ros2 launch felix_bringup mapping.launch.py slam:=false             # base + camera only
ros2 launch felix_bringup mapping.launch.py foxglove:=false         # headless, no UI bridge
ros2 launch felix_bringup mapping.launch.py serial_baudrate:=256000 # if lidar gives no /scan (A2/S1)
ros2 launch felix_bringup mapping.launch.py port:=/dev/ttyUSB0      # custom ROSMASTER port
```

Args: `port` (default `/dev/myserial`), `use_ekf`, `slam`, `camera`, `foxglove`
(all `true`), `serial_baudrate` (`115200` for A1; `256000` for A2/S1). Needs
`ros-humble-robot-localization` (EKF) and the `foxglove_bridge` package installed
(see `BUILD_NOTES.md`).

### Saving & extending a map

With a mapping session still **running**, save from another terminal:

```bash
./save_map.sh                                   # default basename maps/felix_map
./save_map.sh /felix-ai-ros/maps/other          # custom basename
```

This writes **two** formats: the occupancy grid (`.pgm` + `.yaml`, what
localization/navigation load) **and** the slam_toolbox pose-graph (`.posegraph` +
`.data`). The pose-graph is what lets you later **extend** the map — the grid alone
is a one-way export and cannot be resumed. (The serialize step is a service call to
the live `slam_toolbox` node, so save *before* you `Ctrl-C`.)

To **resume and extend** an existing map, park the robot **where the original map
began** (the origin), same heading, launch mapping normally, then deserialize the
saved pose-graph from another terminal:

```bash
ros2 launch felix_bringup mapping.launch.py     # ALWAYS starts a fresh map
./resume_map.sh                                 # given-pose at origin (default)
./resume_map.sh /felix-ai-ros/maps/other        # custom basename
./resume_map.sh /felix-ai-ros/maps/felix_map 1.2 -0.5 1.57   # seed at x y theta (map frame)
./resume_map.sh /felix-ai-ros/maps/felix_map dock            # dock auto-match (see warning)
```

There is **no launch arg** to load a map: the mapping node ignores
`map_file_name` / `map_start_pose` / `map_start_at_dock` — slam_toolbox honours
those only in *localization* mode. Extending the map must go through the
`deserialize_map` service, which is what `resume_map.sh` calls.

By **default** it uses `START_AT_GIVEN_POSE` (match_type 2), seeding the robot at a
known pose in the map frame — origin `(0,0,0)` unless you pass `x y theta`. This
seeds the pose and only scan-matches **locally** (~±20° of heading), so it will
**not** flip; the robot must physically be at that seed pose (the origin if you
give none) or the old graph mis-aligns with new scans. Pass explicit `x y theta`
if you're restarting somewhere else on the existing map.

The optional `dock` argument switches to `START_AT_FIRST_NODE` (match_type 1),
which scan-matches your first scan against the saved first node over a **wide**
orientation search. In rectangular / right-angled rooms that can snap to a 90°
local optimum and build the continuation rotated — avoid it unless the space is
distinctive. Prefer the default given-pose.

Confirm the old map appears in `/map` **and** the live scan lands on the old walls
before driving, extend it, then `./save_map.sh` again.

### Returning to a saved pose (re-homing aid)

`resume_map.sh` (and docking) needs the robot **physically back at the origin with
the same heading**. `home_scan_assist.py` is a lidar "ghost" overlay that helps you
manually drive there. At the pose you want to remember (e.g. p0 on the dock), with
the stack **running** so the `map` frame exists, snapshot the full lidar scan; later
replay it as a *stationary* ghost and drive until the live scan overlaps it.

```bash
# at p0, with mapping/localization running:
python3 home_scan_assist.py save        # → home_scan.json (full scan + p0 pose in map)

# later, after driving away, stack running again:
python3 home_scan_assist.py play        # publishes the frozen ghost on /home_scan
```

In Foxglove (3D panel, **Fixed frame = `map`**) add both `LaserScan` topics:
`/home_scan` (e.g. red — the frozen p0 ghost) and `/scan` (e.g. green — live).
Drive until green sits on top of red; the overlap **is** "back at p0". Because it's
the whole scan, not just front/left/right distances, a rotated robot shows the two
scans fanned apart — so it disambiguates heading too (front/left/right alone are
rotationally ambiguous in boxy rooms, the same 90° flip that bites `resume_map dock`).

The ghost is pinned to a fixed `home_laser` frame (at the captured laser pose in
`map`), so it stays put while the robot moves. No colcon build — runs directly via
`python3`. Flags: `--frame odom` if you're **not** running SLAM/AMCL (no `map`
frame; fine for a quick same-session return, but `odom` drifts/resets so prefer
`map` for returning later); `--file` to use a different fingerprint path (one per
named spot). `home_scan.json` is written as root in the container — `chown
1000:1000 home_scan.json` to commit it from the host.

## Run — localization (drive on a saved map)

Once you've built and **saved** a map (`maps/felix_map.yaml` + `.pgm`), this is the
run-mode twin of mapping: instead of `slam_toolbox` building a map, `nav2_map_server`
serves the saved one and **AMCL** localizes the robot on it, publishing the same
`map→odom` correction. Same one-shot composition (base + EKF + description + RPLIDAR +
camera + Foxglove), so one `Ctrl-C` stops it all. Drive from the Foxglove Teleop panel.

```bash
ros2 launch felix_bringup localization.launch.py                       # default map
ros2 launch felix_bringup localization.launch.py map:=/felix-ai-ros/maps/other.yaml
ros2 launch felix_bringup localization.launch.py camera:=false         # skip camera
```

After launch, **seed AMCL** with a *2D Pose Estimate* in Foxglove/rviz (click the
robot's real location + heading; publishes `/initialpose`), then drive a little so
the particle cloud converges. AMCL starts at the map origin otherwise. Args:
`map` (default `maps/felix_map.yaml`), `port`, `localize`, `camera`, `foxglove`,
`serial_baudrate`. Do **not** run this together with `mapping.launch.py` — both
publish `map→odom`. The map is mecanum-aware: AMCL uses the `OmniMotionModel`.

## Run — navigation (autonomous)

The full autonomy stack: it reuses `localization.launch.py` and adds the `felix_nav`
Nav2 servers on top, so you can send a goal and the robot plans a path and drives
there. Because Felix is holonomic, the controller is **MPPI with `motion_model: Omni`**
— it **strafes** toward goals instead of always turning to face them.

```bash
ros2 launch felix_bringup navigation.launch.py                         # default map
ros2 launch felix_bringup navigation.launch.py map:=/felix-ai-ros/maps/other.yaml
ros2 launch felix_bringup navigation.launch.py navigation:=false       # = localization only
```

To navigate, in Foxglove (3D panel must be in the `map` frame): **(1)** drop a *2D Pose
Estimate* to seed AMCL (publishes `/initialpose`); **(2)** with the *Publish → Pose*
tool, click-drag a destination — but first **set that tool's topic to `/goal_pose`**.
Foxglove defaults it to the ROS 1 name `/move_base_simple/goal`, which Nav2 ignores, so
the pin silently goes nowhere until you change it. Nav2's `bt_navigator` subscribes to
`/goal_pose` natively (no relay needed). The robot then plans (NavFn global), follows
with MPPI + local costmap, and runs recovery behaviors (spin/back-up/wait) if stuck.
The controller publishes `/cmd_vel` straight into the bridge's clamp/watchdog. (rviz
users: its native *Nav2 Goal* tool targets the action directly.)

Speeds in `felix_nav/config/nav2_params.yaml` are **conservative** (vx/vy 0.45 m/s of
the 1.04 max, wz 1.2 of 4.58) — raise them once it behaves. The costmap footprint is
the chassis box (`0.22 × 0.28`). First runs: short goals, open floor, clear of stairs;
`Ctrl-C` drops `/cmd_vel` and the bridge stops the motors.

## Run — natural-language navigation (`felix_llm`)

Teach named places on the map, then tell the **local LLM** (Nemotron via
`llm_server.sh`) to drive there. The agent turns *"go to the kitchen"* into a
`NavigateToPose` goal through Nav2 — it **never writes `/cmd_vel`**, so the
planner and costmaps stay in charge of avoiding obstacles. Three small pieces:

```bash
ros2 launch felix_bringup navigation.launch.py     # localization + Nav2 (gives /amcl_pose + /navigate_to_pose)
./llm_server.sh                                    # serve Nemotron on :8080
ros2 launch felix_llm felix_llm.launch.py          # the NL agent
```

**Teach places** — drive Felix somewhere with teleop, then capture its pose:

```bash
ros2 run felix_llm teach kitchen        # saves current /amcl_pose as "kitchen"
ros2 run felix_llm teach "front door"
```

**Talk to it** — interactive, or one-shot:

```bash
ros2 run felix_llm talk                 # prompt: "go to the kitchen", "where are you", "stop"
ros2 run felix_llm talk go to the kitchen
```

You can also drive it from Foxglove: publish a `std_msgs/String` on `/llm/command`
and watch `/llm/response`. Tools the LLM can call: `go_to`, `list_places`,
`save_place` (so *"save this spot as office"* works), `where_am_i`, `stop`,
`what_do_you_see`, `have_you_seen`.

**Vision** — with perception running (`navigation.launch.py perception:=true`),
ask *"what do you see?"* (live `/perception/detections` labels) or *"have you seen
the cat? where?"*. The agent keeps a passive memory of detections: for *where*, it
uses the map-frame placement from `/perception/objects` when available (floor-
standing objects the lidar can range), else the robot's own pose at the time of
the sighting — then names the nearest saved place: *"Yes, I saw a cat ~10 s ago
near the kitchen."* Without perception running, these tools say so rather than
guessing. Sightings **persist** to `sightings.yaml` (next to your places, under
`install/`, gitignored), so *"have you seen the cat?"* still answers from memory
after a restart — the agent and MCP processes share the file (merge-on-write).

**Autonomous modes** — beyond one-shot commands, the agent runs long-running
behaviors (via a single `BehaviorRunner` in the agent node, so the agent and web
UI never fight over motion). All driving goes through Nav2, so obstacle avoidance
is the planner's job. Progress streams to `/llm/response` (you'll see it in `talk`
/ Foxglove); `stop` ends any of them; `what_am_i_doing` reports the active mode.

| Say | Tool | What it does |
|-----|------|--------------|
| *"find the cat"* | `find` | Drive to where the cat was last seen (persisted sightings) → 360° scan-spin → check the camera; if not there, try the other saved places until found or exhausted. |
| *"patrol"* | `patrol` | Loop through the saved named places, scanning at each, until stopped. |
| *"roam"* / *"drive around"* | `roam` | Wander: pick random reachable points in the mapped free space and let Nav2 drive there avoiding obstacles, forever, until stopped. |

> **Supervise autonomous driving.** These modes drive the robot unattended for
> long stretches. Keep it watched; `stop` (or Ctrl-C on the stack) halts it. Scan-
> spins use the Nav2 `Spin` behavior and are skipped automatically if `/spin`
> isn't up. `roam` needs the `/map` (it's published by the localization/navigation
> stack); `find`/`have_you_seen` need perception (`perception:=true`).

Places live in `src/felix_llm/config/locations.yaml` (name → `{x, y, yaw}` in the
`map` frame); it's symlinked back to source, so taught places persist and can be
hand-edited or committed. Override the endpoint/model with the launch args
`llm_base_url` / `llm_model`, or the file with the `locations_file` node param.

> **Note:** Nemotron is not in llama.cpp's native tool-call handler list, so
> llama-server uses its generic `--jinja` tool path. Validate tool-calling works
> end-to-end once the server is up:
> ```bash
> curl -s http://localhost:8080/v1/chat/completions -H 'Content-Type: application/json' -d '{
>   "model":"nemotron-3-nano-4b",
>   "messages":[{"role":"user","content":"go to the kitchen"}],
>   "tools":[{"type":"function","function":{"name":"go_to","description":"drive to a saved place","parameters":{"type":"object","properties":{"place":{"type":"string"}},"required":["place"]}}}]
> }' | python3 -m json.tool
> ```
> Expect a `tool_calls` entry naming `go_to` with `{"place":"kitchen"}`. If it
> instead answers in prose, lower `temperature` / constrain decoding with a GBNF
> grammar (see `docs/ideation/`), or serve a model with native tool support.

### Browser control via MCP (llama-server web UI)

The same five skills are also exposed as an **MCP server**, so you can drive the
robot by chatting in the **llama-server web UI** instead of the `talk` CLI. The
agent and the MCP server share one implementation (`felix_llm/skills.py`), so
they can't drift.

Once `mcp` is installed, `navigation.launch.py` starts the server automatically
(`mcp:=true` by default, on `:8000`); disable with `mcp:=false` or change the port
with `mcp_port:=...`. To run it standalone instead:

```bash
pip install mcp                                  # one-time, on the Jetson
ros2 run felix_llm mcp                           # streamable-HTTP on :8000/mcp
ros2 run felix_llm mcp --transport sse --port 8000   # or SSE at :8000/sse
```

Then in the web UI's **MCP servers** settings add `http://<jetson>:8000/mcp` (or
the `/sse` URL) — use the host the browser uses (e.g. `http://orin1:8000/mcp`),
not `localhost`. The model can now call `go_to`, `list_places`, `save_place`,
`where_am_i`, `stop` — so "take me to the office" in the browser drives the robot.
Needs Nav2 + localization up, same as the agent.

The server **sends CORS headers itself** (the web UI is a different origin than
`:8000`), so a direct connection works with the proxy toggle **off** — no
`--webui-mcp-proxy` needed. Lock it down on a shared network with
`--cors-origin http://orin1:8080`. (Alternatively, if you prefer routing through
llama-server, serve with `./llm_server.sh --webui-mcp-proxy` and turn the UI's
proxy toggle on instead.)

## Run — perception (YOLO + lidar map placement)

`felix_perception` runs YOLO on the camera and places **floor-standing** detections on
the map using the RPLIDAR. It is **opt-in** (`perception:=true`) on the localization and
navigation stacks and is the perception spine for later work — it does **not** drive the
robot. Two nodes: a **detector** (`/camera/image_raw/compressed` → YOLO on a TensorRT FP16
engine → `vision_msgs/Detection2DArray` + an annotated FPV stream + `CameraInfo`) and a
**fusion** node (detections + `/scan` → `map`-frame markers, by bearing-wedge
intersection). Needs `ros-humble-camera-calibration` + `ros-humble-compressed-image-transport`
(in the Dockerfile) plus Ultralytics/TensorRT (from the base image).

**One-time setup on the Jetson**, in order:

**1 — Build the TensorRT engine** from the YOLO weights. The engine is device/TensorRT-version
specific, so it is built on-device into a cache and never committed; re-run with `--force`
after a JetPack/TensorRT upgrade.

```bash
ros2 run felix_perception build-engine        # yolo11n.pt -> ~/.cache/felix/yolo11n.engine
```

Without an engine the detector still runs on the `.pt` (slower torch path) and says so on
`/perception/status`.

**2 — Calibrate the camera** (headless — no GUI). Print a checkerboard
(`src/felix_perception/calibration/checkerboard_8x6_25mm.pdf`) at **100% / actual size**,
**measure a printed square**, and mount it flat on something rigid. Start the camera, then
collect views and calibrate:

```bash
# terminal A: any stack that publishes the camera
ros2 launch felix_bringup localization.launch.py
# terminal B: collect + calibrate (use your MEASURED square size, in METRES)
ros2 run felix_perception calibrate-camera --square 0.025
```

Add a Foxglove **Image** panel on `/calibration/annotated/compressed` to watch corner
detection; the terminal prints `captured N/40` plus per-axis coverage (X / Y / Size / Skew).
Move the board across the **whole frame** — corners, tilts, near/far — until it hits the
target; it then prints the **RMS reprojection error** and writes `config/camera_info.yaml`
(`Ctrl-C` calibrates early on ≥12 views). Calibrate at the **same resolution** the detector
runs at (the camera's 640×360 bringup default). A clearly-marked placeholder `camera_info`
ships until you do this — every fused map position rides on this calibration.

**Run it** — add `perception:=true` to the localization or navigation stack:

```bash
ros2 launch felix_bringup localization.launch.py perception:=true
ros2 launch felix_bringup navigation.launch.py  perception:=true
```

In Foxglove: an **Image** panel on `/perception/annotated/compressed` shows labelled boxes;
in the **3D** panel (`map` frame) markers on `/perception/objects` drop where floor-standing
objects are. Because the RPLIDAR is a single horizontal plane (~0.115 m up), only objects
crossing it get a map marker — a standing person, chair/table legs; things above the plane
publish as 2D-only (a box, no marker), and map placement needs AMCL localized.
`/perception/status` reports the active backend (`engine` vs `pt`) and inference latency.
Tuning args (on `perception.launch.py`): `weights`, `conf`, `imgsz`, `publish_annotated`,
`require_engine`, `engine_dir`.

## Run — base only

For motion/odometry work without lidar, camera, or UI. `felix.launch.py` composes
the **bridge**, the **ToF** node, the `felix_description` URDF (TF), and — when
`use_ekf:=true` (the default) — the `felix_localization` **EKF** that fuses
`/odom` + `/imu/data` into `odom→base_link`. With the EKF on, the bridge's own
`odom→base_link` TF is suppressed so the two don't fight (`base_link→imu_link`
comes from the URDF).

```bash
ros2 launch felix_bringup felix.launch.py                       # bridge + tof + description + EKF
ros2 launch felix_bringup felix.launch.py port:=/dev/ttyUSB0    # custom serial port
ros2 launch felix_bringup felix.launch.py use_ekf:=false        # raw bridge odom TF, no EKF
```

Or `./start_ros2.sh [port]` — launches `felix.launch.py` in the background and
the keyboard teleop in the foreground (uses `/dev/myserial` by default). `ros2
launch` owns Ctrl-C and shuts everything down cleanly, zeroing the motors.

### Teleop

With `mapping.launch.py` you drive from the Foxglove **Teleop** panel (it
publishes `geometry_msgs/Twist` on `/cmd_vel`) — no terminal teleop required. The
keyboard nodes below still work in their **own terminal** if you prefer (they
read raw keystrokes, which they can't do under `ros2 launch`). The bridge clamps
every command to the configured envelope and stops the motors if commands go
silent (see *Notes*), so all three are safe to drive with.

**Custom node** (`felix_base teleop`) — limits come from `config.yml`:

```bash
ros2 run felix_base teleop
```
```
   u    i    o          i / , : forward / backward
   j    k    l          j / l : rotate left / right
   m    ,    .          u o m . : drive + turn
                        k or space : stop
Hold Shift to strafe:   J / L : strafe left / right

q/z : all speeds ±10%   w/x : linear ±10%   e/c : angular ±10%
Ctrl-C : quit
```

**Standard ROS node** (`teleop_twist_keyboard`) — interchangeable; use its
holonomic keys (or hold Shift) for mecanum strafe (`linear.y`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## config.yml

`src/felix_base/config/config.yml` — symlinked into the install share by
`--symlink-install`, so edits are live (no rebuild).

```yaml
vehicle:
  min_rpm: 30            # below this a wheel stalls (sets the duty deadband)
  max_rpm: 205           # motor OUTPUT (post-gearbox) max, i.e. wheel speed
  wheel_radius: 0.0485   # metres
  wheel_base: 0.19       # front<->rear axle distance, metres
  track_width: 0.265     # left<->right wheel distance, metres
  gear_ratio: 56
  motor_voltage: 12
  velocity_scale: 0.954  # open-loop drive correction (commanded / measured)
  counts_per_rev: 2474   # encoder counts per wheel revolution (drive + odometry)
  motor_map:             # which set_motor index s1..s4 each wheel is wired to
    fl: 1
    fr: 3
    rl: 2
    rr: 4
  motor_sign:            # flip to -1 if a wheel runs backward at +duty
    fl: 1
    fr: 1
    rl: 1
    rr: 1
```

Derived limits (`ros2 run felix_base calibrate limits`):

- **Max linear speed** `v_max = wheel_radius × (max_rpm → rad/s)` ≈ **1.04 m/s**
- **Max yaw rate** `wz_max = v_max / (wheel_base/2 + track_width/2)` ≈ **4.58 rad/s**
- **Min usable speed** ≈ **0.15 m/s** (below this a wheel stalls)
- **Stall deadband** = `min_rpm / max_rpm` ≈ **14.6 %** duty

## Odometry & IMU

The bridge publishes wheel-encoder odometry on `/odom` and broadcasts the
`odom→base_link` TF, plus the board IMU on `/imu/data`. Odometry is **open-loop
dead reckoning** — it drifts, and heading is the least reliable component (wheel
slip is unobserved). The IMU is published so the `felix_localization`
`robot_localization` EKF can fuse gyro yaw on top of the wheel odometry into a
smoother `odom→base_link` — enabled by default in bringup (`use_ekf:=true`).
`felix_description` (URDF/TF) and the RPLIDAR are now in place, so this odometry
feeds `felix_slam` (`slam_toolbox`) in the full-stack launch above.

## Calibration

All steps use the `calibrate` CLI, which talks to the board directly (no ROS
graph). **Put the robot on a stand (wheels off the ground) for `spin`/`drive` the
first time.** Ctrl-C always stops the motors.

```bash
ros2 run felix_base calibrate limits     # print the derived velocity envelope
ros2 run felix_base calibrate spin       # spin each motor s1..s4 -> set motor_map / motor_sign
ros2 run felix_base calibrate encoders   # confirm encoders report (turn wheels by hand)
ros2 run felix_base calibrate cpr --motor 1 --turns 10   # counts per wheel revolution
ros2 run felix_base calibrate drive --vx 0.2 --secs 5    # measured speed -> velocity_scale
ros2 run felix_base calibrate trim --vx 0.2              # per-wheel motor_trim (fix straight-line pull)
```

Put the resulting values into `config.yml` (`motor_map`, `motor_sign`,
`counts_per_rev`, `velocity_scale`, `motor_trim`).

## Notes

- Control is **open-loop** (`set_motor` percent duty); there is no firmware
  velocity PID on this path. `velocity_scale` corrects the average mapping but
  actual speed still varies with load and battery voltage.
- **`/cmd_vel` safety boundary (bridge):** every incoming command is clamped
  per-axis to the configured envelope and stripped of NaN/inf
  (`MecanumKinematics.clamp_body`) before driving the motors — the single guard
  shared by teleop and any future Nav2/SLAM planner. Magnitudes beyond the limits
  are then additionally scaled down **uniformly** (direction preserved) inside
  `body_to_motor`.
- **Watchdog:** because `set_motor()` latches the last command indefinitely, the
  bridge stops the motors if no `/cmd_vel` arrives within `cmd_vel_timeout`
  (default `0.5` s; set to `0` to disable) — failsafe against a crashed publisher
  or dropped link.

## Roadmap

Done: `felix_description` (URDF/TF), `felix_slam` (RPLIDAR + slam_toolbox **mapping**
and `nav2_map_server` + AMCL **localization**), `felix_nav` (**Nav2** autonomy:
NavFn + MPPI holonomic + behaviors), `felix_camera` (CSI → Foxglove FPV), and
`felix_perception` (**YOLO** detector on a TensorRT engine + **lidar-fused** map
placement, `perception:=true`), and `felix_llm` (**natural-language control**: a
local-LLM agent + MCP server for teach-a-place, go-to, vision Q&A, and
find/patrol/roam) — composed by `mapping` / `localization` / `navigation`
one-shots.

- **Perception next:** Home Assistant/MQTT output, a record-while-driving dataset to
  fine-tune YOLO for the house, and fiducial-marker docking (see `docs/ideation/`).
- **WebRTC video** → `felix_streaming` (likely its own process/container).
- **Nav2 tuning**: raise MPPI speed limits and tune critic weights from the
  conservative defaults once autonomous driving is verified on hardware.

## llama-cpp

```
llama-completion -m /data/models/llm/NVIDIA-Nemotron3-Nano-4B-Q4_K_M.gguf -ngl 999 --jinja -n 220 -p "In one sentence, what is a mecanum wheel?"
```