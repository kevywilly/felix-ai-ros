# CLAUDE.md

Guidance for AI agents working in this repository.

## What this is

Base driver, teleop, and a full **autonomy stack** (odometry + IMU → EKF → SLAM /
AMCL → **Nav2**) for a **custom 4-wheel mecanum robot** on a Yahboom ROSMASTER
board (NVIDIA Jetson Orin, ROS 2 Humble), organized as a colcon workspace. See
`README.md` for full docs.

## Workspace layout

The repo root **is** the colcon workspace (it reuses the host dir bind-mounted
into the Docker container — no separate `felix_ws`). Packages live under `src/`:

- **`felix_base`** — base driver: the hardware bridge (`/cmd_vel` → motors,
  open-loop), wheel-encoder **odometry** + `odom→base_link` TF, **IMU**
  publishing, the ToF array, keyboard teleop, the mecanum kinematics, and
  `config/config.yml`.
- **`felix_localization`** — `robot_localization` EKF fusing `/odom` (wheel
  `vx`/`vy`) + `/imu/data` (gyro yaw rate) into a smooth `odom→base_link`.
- **`felix_description`** — URDF/TF: publishes `base_footprint→base_link→{imu_link,
  laser, camera_link}` on `/tf_static` (the EKF, SLAM, and Nav2 consume it).
- **`felix_slam`** — RPLIDAR driver + **mapping** (`slam_toolbox`) **and
  localization** (`nav2_map_server` + **AMCL** on a saved map → `map→odom`).
- **`felix_nav`** — **Nav2** autonomy: NavFn planner, **MPPI** holonomic
  controller, recovery behaviors, BT navigator → `/cmd_vel`.
- **`felix_camera`** — CSI camera → compressed image (Foxglove FPV).
- **`felix_bringup`** — top-level launch composition + params, depending on all of
  the above. Four stacks: `felix.launch.py` (base alone), `mapping.launch.py`,
  `localization.launch.py`, `navigation.launch.py`. Future subsystems
  (`felix_perception`, `felix_streaming`) compose in here.

Build & run (from the repo root):

```bash
colcon build --symlink-install
source install/setup.bash
./start_ros2.sh [port]          # launch (bridge+tof, bg) + teleop (fg)
# or piecemeal:
ros2 launch felix_bringup felix.launch.py port:=/dev/myserial
ros2 run felix_base teleop
```

ROS-side code imports `felix_base.*` (the old `lib.*` / run-from-root layout is
gone). See `BUILD_NOTES.md` for the dev loop and the required setuptools pin.

## Critical context (read before changing motion code)

- **The chassis is custom, not a stock Yahboom frame.** Do NOT use the firmware's
  `set_car_motion()` / `set_car_run()` / `car_type` kinematics — they assume
  built-in Yahboom geometry that does not match this robot. Motion goes through
  **our own mecanum kinematics** (`felix_base/kinematics.py`) →
  `Rosmaster.set_motor()` (per-wheel percent duty, **open loop**, no firmware PID).
- **`config.yml` is the single source of truth** for geometry, motor specs, and
  calibration. It lives at `src/felix_base/config/config.yml` and is resolved via
  the package share dir (symlinked back to source by `--symlink-install`). The
  bridge, teleop, and `calibrate` all read limits/kinematics from it. Never
  hardcode wheel geometry or velocity limits elsewhere.

## Conventions

- Velocity frame is ROS REP-103: `vx>0` forward, `vy>0` left, `wz>0` CCW.
- Topics: in `/cmd_vel` (`geometry_msgs/Twist`); out `/odom` (`nav_msgs/Odometry`)
  and `/imu/data` (`sensor_msgs/Imu`); TF `odom→base_link`. Teleop publishes
  `/cmd_vel`, the bridge subscribes. Keep them in sync if you change them.
- Wheel order everywhere is `(fl, fr, rl, rr)` — see `felix_base.kinematics.WHEELS`.
- `set_motor()` takes 4 ints in `[-100, 100]` (percent duty), order `s1..s4`.
  The wheel→index wiring and forward-direction signs live in
  `config.yml` (`motor_map` / `motor_sign`), not in code.

## Odometry / IMU

- `felix_base/odometry.py` integrates encoder deltas → planar pose via forward
  kinematics (`kinematics.wheels_to_body`, the exact inverse of `body_to_wheels`).
  This is open-loop **dead reckoning**: it drifts, and heading (`theta`) is the
  least trustworthy component because lateral/rotational wheel slip is unobserved.
- The **bridge owns the single serial connection**, so it is the one node that
  reads encoders + IMU (you cannot open `/dev/myserial` twice). It calls
  `create_receive_threading()` once at startup, then a timer polls and publishes.
- IMU comes from `get_gyroscope_data()` (rad/s) / `get_accelerometer_data()`
  (m/s², MPU9250 firmware path) / `get_imu_attitude_data()` (fused rad). If the
  board is an ICM20948 the raw ratios differ — verify by rotating and watching
  `angular_velocity.z`.
- **Fusion** is the `felix_localization` EKF (`config/ekf.yaml`): wheel `vx`/`vy`
  + IMU gyro yaw-rate → `odom→base_link`. When the EKF runs (`use_ekf:=true`,
  the default), `felix_bringup` sets the bridge's `publish_tf:=false` so they
  don't both publish that transform. The `base_link→imu_link` (and `→laser`)
  transforms now come from the `felix_description` URDF on `/tf_static` (this
  replaced the temporary identity `static_transform_publisher` we used early on).
  Needs `ros-humble-robot-localization` installed. EKF runs at 50 Hz so the
  `odom→base_link` TF stays fresh for AMCL.

## Calibration values (in config.yml)

- `counts_per_rev` — encoder counts per wheel rev (`calibrate cpr`). Also used by
  odometry to convert encoder deltas to wheel radians.
- `velocity_scale` — open-loop correction, `commanded / measured`
  (`calibrate drive`). Applied in `MecanumKinematics.body_to_motor` by
  pre-multiplying the command. 1.0 = no correction. NOT applied to odometry
  (which measures real rotation).
- `motor_map` / `motor_sign` — wiring + direction (`calibrate spin`). Used by both
  `body_to_motor` (drive) and `encoder_deltas_to_wheel_rads` (odometry).
- `motor_trim` — per-wheel open-loop gain trim. Drive only; NOT applied to odometry.

## Gotchas

- `felix_base/rosmaster.py` is vendor hardware code — avoid editing it; wrap it.
- Reading encoders/IMU requires `bot.create_receive_threading()` first (the
  bridge does this in `__init__`).
- `--symlink-install` needs `setuptools<80` on this image (81 fails with
  `option --editable not recognized`) — see `BUILD_NOTES.md`; pin it in the
  Dockerfile.
- `start_ros2.sh` sources ROS + `install/` inside a `set +u` / `set -u` block
  (ROS `setup.bash` trips `nounset`). Keep that if editing the script.
- Open-loop means real speed drifts with battery/load; `velocity_scale` only
  corrects the average. Don't expect exact velocities without encoder feedback.
- **Foxglove nav pins are plain topic publishes** (no rviz tools). Start pin =
  *2D Pose Estimate* → `/initialpose` (AMCL). Goal pin = *Publish → Pose*, but its
  topic **defaults to the ROS 1 `/move_base_simple/goal`** — change it to
  `/goal_pose`, which `bt_navigator` subscribes to natively (do NOT add a relay).
  The 3D panel must be in the `map` frame for either to land correctly.
- AMCL's `Failed to transform initial pose in time ... extrapolation into the
  future` on a start pin is **cosmetic** — it falls back to identity and sets the
  pose anyway. Verify via `/amcl_pose` / the particle cloud, not the log.

## Quick checks

```bash
ros2 run felix_base calibrate limits     # derived velocity envelope (no hardware)
python3 -m py_compile src/felix_base/felix_base/*.py
colcon build --symlink-install
```

## Hardware / roadmap

- ROSMASTER board on `/dev/myserial`; **RPLIDAR A2 M12 on `/dev/rplidar` @
  256000 baud** (the `felix_slam` driver node; 256000 is now the launch default).
  Mounted +83 mm forward / +75 mm up from `base_link` (`base_link→laser` in
  `felix_description`). Encoders confirmed reporting; the board has an IMU.
- **Done:** `felix_description` (URDF/TF), `felix_slam` (RPLIDAR + slam_toolbox
  mapping and `nav2_map_server` + AMCL localization), `felix_nav` (Nav2 autonomy:
  NavFn + MPPI holonomic + behaviors), `felix_camera` (CSI → Foxglove FPV).
- **Planned:** `felix_perception` (YOLO → `vision_msgs`), `felix_streaming` (WebRTC
  video). Nav2 tuning: raise MPPI speed limits / tune critics off the conservative
  defaults once autonomy is verified on hardware.

## Safety

For any test that drives motors (`calibrate spin`/`drive`, `start_ros2.sh`), the
robot should be on a stand (wheels off the ground) the first time. All entry
points stop the motors on exit / Ctrl-C — preserve that behavior.
