# felix-ai-ros

Keyboard teleoperation and base driver for a **custom 4-wheel mecanum robot**
driven by a Yahboom ROSMASTER board on an NVIDIA Jetson Orin (ROS 2 Humble).

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
| `felix_bringup` | Top-level launch composition + params (depends on `felix_base`). |

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
teleop ──/cmd_vel (Twist)──► bridge_node ──► set_motor() ──► motors
                              bridge_node ──/odom (Odometry)──►   + TF odom→base_link
                              bridge_node ──/imu/data (Imu)──►
                              tof_array_node ──/tof (Int16MultiArray)──►
```

## Requirements

- ROS 2 Humble (`source /opt/ros/humble/setup.bash`)
- Python 3 with `rclpy`, `pyserial`, `pyyaml`
- `setuptools<80` for `--symlink-install` (see `BUILD_NOTES.md`)
- The ROSMASTER board on a serial port (default `/dev/myserial`)

## Build & run

```bash
cd /felix-ai-ros
colcon build --symlink-install
source install/setup.bash

./start_ros2.sh                 # uses /dev/myserial
./start_ros2.sh /dev/ttyUSB0    # custom serial port
```

`start_ros2.sh` launches the bridge + ToF nodes in the background (via
`ros2 launch felix_bringup felix.launch.py`) and the keyboard teleop in the
foreground. `ros2 launch` owns Ctrl-C and shuts everything down cleanly, zeroing
the motors. Or run pieces directly:

```bash
ros2 launch felix_bringup felix.launch.py port:=/dev/myserial   # bridge + tof
ros2 run felix_base teleop                                      # teleop (own terminal)
ros2 run felix_base calibrate limits                            # calibration CLI
```

### Keyboard controls

```
   u    i    o          i / , : forward / backward
   j    k    l          j / l : rotate left / right
   m    ,    .          u o m . : drive + turn
                        k or space : stop
Hold Shift to strafe:   J / L : strafe left / right

q/z : all speeds ±10%   w/x : linear ±10%   e/c : angular ±10%
Ctrl-C : quit
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
slip is unobserved). The IMU is published so a `robot_localization` EKF can fuse
gyro yaw on top of the wheel odometry; that fusion is the planned next step
before SLAM and is **not** done yet.

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
- Commands beyond the chassis limits are scaled down **uniformly** (direction
  preserved) inside `MecanumKinematics.body_to_motor`.

## Roadmap

- **RPLIDAR** (on `/dev/rplidar`) → `felix_slam` (slam_toolbox), once
  `felix_description` (URDF/TF) and EKF-fused odometry are in place.
- **YOLO** → `felix_perception` (`vision_msgs/Detection2DArray`).
- **WebRTC video** → `felix_streaming` (likely its own process/container).
