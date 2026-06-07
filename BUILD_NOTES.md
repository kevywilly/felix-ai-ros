# ROS 2 workspace build notes

The repo root is a colcon workspace (it reuses the host dir already bind-mounted
into the container — no separate `felix_ws`). Packages live under `src/`:

- **`felix_base`** — base driver: hardware bridge, keyboard teleop, ToF array,
  and the open-loop mecanum kinematics + `config/config.yml`.
- **`felix_bringup`** — top-level launch composition + params (depends on
  `felix_base`). Future subsystems (description, slam, perception, streaming)
  get composed in here too.

## One-time / after setup.py or package.xml changes

```bash
colcon build --symlink-install
```

`--symlink-install` keeps the dev loop fast:
- `felix_base/config/config.yml` is **symlinked** into the install share → edits
  are live, no rebuild. It remains the single source of truth (resolved via the
  package share dir by `felix_base.kinematics._default_config_path()`).
- `.py` files are **hardlinked** src↔build → live as long as your editor writes
  in place. If your editor does atomic save (write-temp + rename), the link
  breaks; just re-run `colcon build` (~3 s) to re-sync. When in doubt, rebuild.

## Dockerfile pin (REQUIRED — do this in the image)

ROS 2 Humble's `--symlink-install` needs setuptools < 80. The base image here
shipped setuptools 81, which fails with `option --editable not recognized`. It
was pinned in the running container with:

```bash
pip3 install "setuptools<80"
```

**Add the same pin to the Dockerfile** so it survives image rebuilds, e.g.:

```dockerfile
RUN pip3 install "setuptools<80"
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Hardware bridge + ToF (background) and keyboard teleop (foreground):
./start_ros2.sh /dev/myserial

# Or piecemeal:
ros2 launch felix_bringup felix.launch.py port:=/dev/myserial   # bridge + tof
ros2 run felix_base teleop                                       # teleop (own terminal)
ros2 run felix_base calibrate limits                             # calibration CLI
```

`ros2 launch` owns SIGINT and shuts the bridge + ToF down cleanly on Ctrl-C —
this is what replaces the manual kill/wait dance in the old `start.sh`. Teleop is
not in the launch file because it needs a real terminal for raw keystrokes.

## Post-verification cleanup (after testing start_ros2.sh on the robot)

The old entry points are kept as a working fallback until the package is verified
on hardware. Once `./start_ros2.sh` is confirmed driving the robot, these become
redundant and can be removed:

- `main.py`, `calibrate.py`            (now `felix_base/teleop.py`, `calibrate.py`)
- `nodes/`                              (now `felix_base/bridge_node.py`, `tof_array_node.py`)
- `lib/`                                (now `felix_base/kinematics.py`, `rosmaster.py`)
- `config.yml` (root)                   (now `src/felix_base/config/config.yml`)
- `start.sh`                            (replaced by `start_ros2.sh`)

Then update `CLAUDE.md` / `README.md` to point at the package layout and the
`src/felix_base/config/config.yml` path.

## Planned packages (created when each subsystem starts — don't scaffold empty)

- `felix_description` — URDF/xacro + robot_state_publisher (needed for TF / SLAM)
- `felix_slam`        — config + launch wrapping slam_toolbox / rtabmap
- `felix_perception`  — YOLO node → vision_msgs/Detection2DArray
- `felix_streaming`   — WebRTC / video service (likely its own process/container)

Next capability: wheel-encoder **odometry + odom→base_link TF** in `felix_base`
(encoders confirmed reporting on this chassis) — the prerequisite SLAM blocks on.
