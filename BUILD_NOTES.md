# ROS 2 workspace build notes

The repo root is a colcon workspace (it reuses the host dir already bind-mounted
into the container — no separate `felix_ws`). Packages live under `src/`:

- **`felix_base`** — base driver: hardware bridge, keyboard teleop, ToF array,
  the open-loop mecanum kinematics + `config/config.yml`, wheel odometry, IMU.
- **`felix_localization`** — `robot_localization` EKF fusing `/odom` + `/imu/data`
  → `odom→base_link` (`config/ekf.yaml`). Needs `ros-humble-robot-localization`.
- **`felix_bringup`** — top-level launch composition + params (depends on
  `felix_base` + `felix_localization`). Future subsystems (description, slam,
  perception, streaming) get composed in here too.

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

## Pre-package cleanup (done)

The old top-level entry points were removed once the package was verified driving
on hardware — they are fully superseded by `felix_base` / `felix_bringup`:

- `main.py`, `calibrate.py`  → `felix_base/teleop.py`, `felix_base/calibrate.py`
- `nodes/`                   → `felix_base/bridge_node.py`, `felix_base/tof_array_node.py`
- `lib/`                     → `felix_base/kinematics.py`, `felix_base/rosmaster.py`
- `config.yml` (root)        → `src/felix_base/config/config.yml` (the only one)
- `start.sh`                 → `start_ros2.sh`

`CLAUDE.md` / `README.md` point at the package layout. The repo root now holds
only `src/`, `start_ros2.sh`, and the docs (plus gitignored `build/install/log`).

## EKF (felix_localization) — prerequisite

Install once on the robot (and add to the Dockerfile):

```bash
sudo apt install ros-humble-robot-localization
```

Then `./start_ros2.sh` runs the EKF by default (`use_ekf:=true`). It fuses wheel
`vx`/`vy` + IMU gyro yaw-rate → `odom→base_link`; the bridge's `publish_tf` is
set false so the EKF owns that transform. Run without it via
`ros2 launch felix_bringup felix.launch.py use_ekf:=false` (bridge publishes the
raw odom TF instead).

## Planned packages (created when each subsystem starts — don't scaffold empty)

- `felix_description` — URDF/xacro + robot_state_publisher (needed for TF / SLAM;
  also lands the real base_link→imu_link transform replacing the temp static one)
- `felix_slam`        — config + launch wrapping slam_toolbox / rtabmap
- `felix_perception`  — YOLO node → vision_msgs/Detection2DArray
- `felix_streaming`   — WebRTC / video service (likely its own process/container)
