# CLAUDE.md

Guidance for AI agents working in this repository.

## What this is

Keyboard teleop for a **custom 4-wheel mecanum robot** on a Yahboom ROSMASTER
board (NVIDIA Jetson Orin, ROS 2 Humble). See `README.md` for full docs.

## Critical context (read before changing motion code)

- **The chassis is custom, not a stock Yahboom frame.** Do NOT use the firmware's
  `set_car_motion()` / `set_car_run()` / `car_type` kinematics — they assume
  built-in Yahboom geometry that does not match this robot. Motion goes through
  **our own mecanum kinematics** (`lib/kinematics.py`) → `Rosmaster.set_motor()`
  (per-wheel percent duty, **open loop**, no firmware PID).
- **`config.yml` is the single source of truth** for geometry, motor specs, and
  calibration. `main.py`, the bridge node, and `calibrate.py` all read limits and
  kinematics from it. Never hardcode wheel geometry or velocity limits elsewhere.
- All Python is run from the **repository root** so `from lib.xxx import ...`
  resolves (e.g. `python3 nodes/rosmaster_bridge_node.py`). `start.sh` sets
  `PYTHONPATH` and `cd`s to root for this reason.

## Conventions

- Velocity frame is ROS REP-103: `vx>0` forward, `vy>0` left, `wz>0` CCW.
- The topic is `/cmd_vel` (`geometry_msgs/Twist`); `main.py` publishes, the bridge
  subscribes. Keep them in sync if you change it.
- Wheel order everywhere is `(fl, fr, rl, rr)` — see `lib.kinematics.WHEELS`.
- `set_motor()` takes 4 ints in `[-100, 100]` (percent duty), order `s1..s4`.
  The wheel→index wiring and forward-direction signs live in
  `config.yml` (`motor_map` / `motor_sign`), not in code.

## Calibration values (in config.yml)

- `counts_per_rev` — encoder counts per wheel rev (`calibrate.py cpr`).
- `velocity_scale` — open-loop correction, `commanded / measured`
  (`calibrate.py drive`). Applied in `MecanumKinematics.body_to_motor` by
  pre-multiplying the command. 1.0 = no correction.
- `motor_map` / `motor_sign` — wiring + direction (`calibrate.py spin`).

## Gotchas

- `lib/rosmaster.py` is vendor hardware code — avoid editing it; wrap it instead.
- Reading encoders/odometry requires `bot.create_receive_threading()` first.
- `start.sh` uses `set -euo pipefail`; ROS `setup.bash` trips `nounset`, so it is
  sourced inside a `set +u` / `set -u` block. Keep that if editing the script.
- Open-loop means real speed drifts with battery/load; `velocity_scale` only
  corrects the average. Don't expect exact velocities without encoder feedback.

## Quick checks

```bash
python3 lib/kinematics.py            # print derived velocity envelope
python3 -m py_compile main.py calibrate.py lib/kinematics.py nodes/rosmaster_bridge_node.py
python3 calibrate.py limits          # same envelope, via the CLI
```

## Safety

For any test that drives motors (`calibrate.py spin`/`drive`, `start.sh`), the
robot should be on a stand (wheels off the ground) the first time. All entry
points stop the motors on exit / Ctrl-C — preserve that behavior.
