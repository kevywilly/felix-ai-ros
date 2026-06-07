# felix-ai-ros

Keyboard teleoperation for a **custom 4-wheel mecanum robot** driven by a Yahboom
ROSMASTER board on an NVIDIA Jetson Orin (ROS 2 Humble).

Because the chassis is custom (not a stock Yahboom frame), the board firmware's
built-in kinematics (`set_car_motion`) do not apply. Instead this project
computes mecanum kinematics from the robot's real geometry in `config.yml` and
drives the four motors directly, open-loop, via `set_motor()`.

## Architecture

```
keyboard ──► main.py ──/cmd_vel──► rosmaster_bridge_node ──► lib/kinematics ──► set_motor() ──► STM32 ──► motors
            (teleop)    (Twist)      (subscriber)            (mecanum IK)        (Rosmaster)
```

| File | Role |
|------|------|
| `main.py` | Keyboard teleop node. Reads keystrokes, publishes `geometry_msgs/Twist` on `/cmd_vel`. Speed limits come from `config.yml`. |
| `nodes/rosmaster_bridge_node.py` | Subscribes to `/cmd_vel`, converts the body velocity to per-wheel motor commands via `lib/kinematics`, and sends them with `set_motor()`. |
| `lib/kinematics.py` | Mecanum inverse kinematics + velocity limits + wheel→percent mapping. Single source of truth, loads `config.yml`. |
| `lib/rosmaster.py` | Yahboom hardware driver (serial protocol to the ROSMASTER board). Vendor code. |
| `calibrate.py` | Calibration & verification tool (see below). |
| `config.yml` | All robot geometry, motor specs, and calibration values. **The only place to edit them.** |
| `start.sh` | Launches the bridge node (background) + teleop (foreground). |

## Requirements

- ROS 2 Humble (`source /opt/ros/humble/setup.bash`)
- Python 3 with `rclpy`, `pyserial`, `pyyaml`
- The ROSMASTER board on a serial port (default `/dev/myserial`)

## Running

From the repository root:

```bash
./start.sh                 # uses /dev/myserial
./start.sh /dev/ttyUSB0    # custom serial port
```

The bridge starts in the background; the teleop owns the terminal. Ctrl-C stops
both and zeroes the motors.

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

```yaml
vehicle:
  min_rpm: 30            # below this a wheel stalls (sets the duty deadband)
  max_rpm: 205           # motor OUTPUT (post-gearbox) max, i.e. wheel speed
  wheel_radius: 0.0485   # metres
  wheel_base: 0.19       # front<->rear axle distance, metres
  track_width: 0.265     # left<->right wheel distance, metres
  gear_ratio: 56
  motor_voltage: 12
  velocity_scale: 0.954  # open-loop correction (commanded / measured)
  counts_per_rev: 2474   # encoder counts per wheel revolution
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

Derived limits (print with `python3 lib/kinematics.py`):

- **Max linear speed** `v_max = wheel_radius × (max_rpm → rad/s)` ≈ **1.04 m/s**
- **Max yaw rate** `wz_max = v_max / (wheel_base/2 + track_width/2)` ≈ **4.58 rad/s**
- **Min usable speed** ≈ **0.15 m/s** (below this a wheel stalls)
- **Stall deadband** = `min_rpm / max_rpm` ≈ **14.6 %** duty

## Calibration

All steps use `calibrate.py`, which talks to the board directly (no ROS graph).
**Put the robot on a stand (wheels off the ground) for `spin`/`drive` the first
time.** Ctrl-C always stops the motors.

```bash
python3 calibrate.py limits     # print the derived velocity envelope
```

### 1. Wheel order & direction — `spin`

```bash
python3 calibrate.py spin
```

Spins each motor index `s1..s4` one at a time. Note which physical wheel turns
and whether it goes **forward**. Then edit `config.yml`:

- `motor_map`: set `fl/fr/rl/rr` to the `s`-index that drove that wheel.
- `motor_sign`: set to `-1` for any wheel that ran **backward** at `+duty`.

### 2. Encoder counts-per-revolution — `cpr`

```bash
python3 calibrate.py encoders            # confirm encoders report (turn wheels by hand)
python3 calibrate.py cpr --motor 1 --turns 10   # turn that wheel exactly 10 revs by hand
```

Put the reported value into `config.yml` as `counts_per_rev`. Sanity check:
`counts_per_rev / gear_ratio` should equal your encoder's counts-per-motor-rev.

### 3. Real-world speed correction — `drive`

```bash
python3 calibrate.py drive --vx 0.2 --secs 5
```

With `counts_per_rev` set, this prints the **measured** speed and a
`scale (commanded / measured)`. Put that number into `config.yml` as
`velocity_scale`. After that, commanded m/s should match real-world m/s.
(No tape measure needed once encoders are calibrated; otherwise measure the
travelled distance by hand: `measured_speed = distance / elapsed`.)

Re-run `drive` after setting `velocity_scale` to confirm the scale is ~1.0.

## Notes

- Control is **open-loop** (`set_motor` percent duty); there is no firmware
  velocity PID on this path. `velocity_scale` corrects the average mapping but
  actual speed still varies with load and battery voltage.
- Commands beyond the chassis limits are scaled down **uniformly** (direction
  preserved) inside `MecanumKinematics.body_to_motor`.
