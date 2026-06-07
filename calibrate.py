#!/usr/bin/env python3
"""
Calibration helper for the custom Felix mecanum chassis.

Talks to the ROSMASTER board directly (no ROS graph needed) so you can verify,
in isolation, that the numbers derived from config.yml match reality.

Run from the repository root:

    python3 calibrate.py limits        # print theoretical velocity envelope
    python3 calibrate.py encoders      # live encoder monitor (spin wheels by hand)
    python3 calibrate.py spin          # spin each motor 1-by-1 to find order + signs
    python3 calibrate.py cpr           # measure encoder counts-per-wheel-revolution
    python3 calibrate.py drive --vx 0.2 --secs 5   # timed drive for tape-measure check
    python3 calibrate.py trim --vx 0.2             # suggest per-wheel motor_trim (fix pull)

SAFETY: for `spin` and `drive`, put the robot up on a stand (wheels off the
ground) the first time, and keep the area clear. `trim` is a straight-line floor
test -- give it clear space ahead. Ctrl-C always stops the motors.
"""
import argparse
import sys
import time

from lib.rosmaster import Rosmaster
from lib.kinematics import MecanumKinematics, WHEELS

DEFAULT_PORT = "/dev/myserial"


def connect(port):
    bot = Rosmaster(com=port)
    bot.create_receive_threading()   # required for get_motor_encoder()/get_motion_data()
    time.sleep(0.5)
    return bot


def stop(bot):
    bot.set_motor(0, 0, 0, 0)


# --------------------------------------------------------------------------- #
# limits
# --------------------------------------------------------------------------- #
def cmd_limits(args):
    print(MecanumKinematics(args.config).describe_limits())


# --------------------------------------------------------------------------- #
# encoders: live monitor so you can see which index moves and in which sign
# --------------------------------------------------------------------------- #
def cmd_encoders(args):
    bot = connect(args.port)
    print("Reading encoders. Turn each wheel BY HAND and watch which m1..m4 "
          "changes and its sign.\nCtrl-C to stop.\n")
    try:
        base = bot.get_motor_encoder()
        if base == (0, 0, 0, 0):
            print("Initial read is all zeros -- if these never change while you "
                  "turn the wheels, this board/firmware does not report encoders "
                  "on this chassis. Use the `drive` (tape-measure) flow instead.\n")
        while True:
            m = bot.get_motor_encoder()
            d = tuple(m[i] - base[i] for i in range(4))
            print(f"\rraw={m}  delta={d}   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nDone.")


# --------------------------------------------------------------------------- #
# spin: drive one motor at a time to identify physical wheel + forward sign
# --------------------------------------------------------------------------- #
def cmd_spin(args):
    bot = connect(args.port)
    pct = args.percent
    print(f"Spinning each motor at +{pct}% for {args.secs}s, one at a time.\n"
          f"Note which physical wheel turns and whether it goes FORWARD.\n"
          f"If a wheel runs backward at +{pct}%, set its motor_sign to -1 in config.yml.\n")
    try:
        for idx in range(1, 5):
            speeds = [0, 0, 0, 0]
            speeds[idx - 1] = pct
            print(f"--> motor index s{idx} = +{pct}% (others 0)")
            had = bot.get_motor_encoder()
            bot.set_motor(*speeds)
            time.sleep(args.secs)
            stop(bot)
            now = bot.get_motor_encoder()
            delta = tuple(now[i] - had[i] for i in range(4))
            print(f"    encoder delta during this spin: {delta}")
            time.sleep(0.5)
    finally:
        stop(bot)
    print("\nFill config.yml vehicle.motor_map (which s-index is fl/fr/rl/rr) "
          "and vehicle.motor_sign accordingly.")


# --------------------------------------------------------------------------- #
# cpr: counts per wheel revolution, measured by hand-turning a known # of turns
# --------------------------------------------------------------------------- #
def cmd_cpr(args):
    bot = connect(args.port)
    idx = args.motor
    print(f"Measuring counts-per-revolution on motor s{idx}.")
    print(f"Turn that wheel EXACTLY {args.turns} full revolutions by hand, "
          f"then press Enter (Ctrl-C to abort).")
    base = bot.get_motor_encoder()
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted.")
        return
    now = bot.get_motor_encoder()
    delta = now[idx - 1] - base[idx - 1]
    if delta == 0:
        print("No change -- encoders not reporting for this motor.")
        return
    cpr = abs(delta) / args.turns
    print(f"\nencoder delta = {delta} over {args.turns} rev")
    print(f"--> counts per wheel revolution (CPR) ~= {cpr:.1f}")
    print(f"    (use this to convert encoder counts to distance/speed: "
          f"metres = counts / CPR * 2*pi*wheel_radius)")


# --------------------------------------------------------------------------- #
# drive: command a body velocity for N seconds, report encoder-based speed if any
# --------------------------------------------------------------------------- #
def cmd_drive(args):
    kin = MecanumKinematics(args.config)
    bot = connect(args.port)
    motors = kin.body_to_motor(args.vx, args.vy, args.wz)

    print(f"Commanding vx={args.vx} m/s, vy={args.vy} m/s, wz={args.wz} rad/s "
          f"for {args.secs}s")
    print(f"  -> set_motor{motors}")
    if args.vx and not args.vy and not args.wz:
        print(f"  -> expected straight-line distance: "
              f"{args.vx * args.secs:.3f} m (measure with a tape and compare)")

    base = bot.get_motor_encoder()
    t0 = time.time()
    try:
        bot.set_motor(*motors)
        while time.time() - t0 < args.secs:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop(bot)
    elapsed = time.time() - t0
    now = bot.get_motor_encoder()
    delta = tuple(now[i] - base[i] for i in range(4))
    print(f"\nelapsed={elapsed:.2f}s  encoder delta={delta}")

    cpr = args.cpr or kin.counts_per_rev
    if cpr:
        avg_counts = sum(abs(d) for d in delta) / 4.0
        dist = avg_counts / cpr * kin.wheel_circumference
        measured = dist / elapsed if elapsed else 0.0
        print(f"avg wheel-path distance={dist:.3f} m (cpr={cpr:g})  "
              f"-> measured speed ~= {measured:.3f} m/s")
        if args.vx and not args.vy and not args.wz and measured > 0:
            print(f"scale (commanded/measured) = {args.vx / measured:.3f}  "
                  f"<- put this in config.yml as vehicle.velocity_scale if != ~1.0")
    else:
        print("No counts_per_rev configured; measure travelled distance with a tape:")
        print("    measured_speed = distance_m / elapsed ; scale = commanded / measured")


# --------------------------------------------------------------------------- #
# trim: drive straight, compare per-wheel encoder travel, suggest motor_trim
# --------------------------------------------------------------------------- #
def _trim_run(bot, kin, vx, secs):
    """One straight drive; returns per-wheel absolute encoder counts (fl/fr/rl/rr)."""
    motors = kin.body_to_motor(vx, 0.0, 0.0)
    base = bot.get_motor_encoder()
    t0 = time.time()
    try:
        bot.set_motor(*motors)
        while time.time() - t0 < secs:
            time.sleep(0.05)
    finally:
        stop(bot)
    now = bot.get_motor_encoder()
    # Map raw motor-index deltas back to logical wheels (fl/fr/rl/rr).
    return {w: abs(now[kin.motor_map[w] - 1] - base[kin.motor_map[w] - 1])
            for w in WHEELS}


def cmd_trim(args):
    kin = MecanumKinematics(args.config)
    bot = connect(args.port)
    runs = max(1, args.runs)

    print(f"Measuring per-wheel travel over {runs} run(s) of {args.secs}s at "
          f"|vx|={args.vx} m/s.")
    if runs > 1:
        print("Direction alternates each run (fwd/back) so the robot stays near "
              "its start; counts are magnitudes, so direction does not bias them.")
    print("Robot should be ON THE FLOOR with clear space (this is a straight-"
          "line test, not a stand test). Ctrl-C stops the motors.\n")

    # Average across runs to smooth out encoder jitter, slip, and battery sag.
    per_run = []
    try:
        for i in range(runs):
            vx = args.vx if i % 2 == 0 else -args.vx   # alternate to stay in place
            c = _trim_run(bot, kin, vx, args.secs)
            per_run.append(c)
            print(f"  run {i + 1}/{runs} (vx={vx:+.2f}): "
                  + "  ".join(f"{w}={c[w]}" for w in WHEELS))
            if i < runs - 1:
                time.sleep(args.pause)               # settle between runs
    except KeyboardInterrupt:
        stop(bot)
        if not per_run:
            print("\nAborted before any run completed.")
            return
        print(f"\nInterrupted; using the {len(per_run)} completed run(s).")

    counts = {w: sum(r[w] for r in per_run) / len(per_run) for w in WHEELS}
    print("\nmean travel (counts): "
          + "  ".join(f"{w}={counts[w]:.0f}" for w in WHEELS))
    if len(per_run) > 1:
        # Spread per wheel as a sanity flag on run-to-run consistency.
        spread = {w: (max(r[w] for r in per_run) - min(r[w] for r in per_run))
                  for w in WHEELS}
        worst = max(spread[w] / counts[w] for w in WHEELS if counts[w])
        print(f"max run-to-run spread: {worst * 100:.0f}% of mean"
              + ("  (high -- add runs or check the surface/battery)"
                 if worst > 0.15 else ""))

    if min(counts.values()) == 0:
        print("\nAt least one wheel reported zero travel -- encoders are not "
              "usable here. Eyeball which side leads instead: if the robot "
              "pulls LEFT, lower fr/rr in config.yml (motor_trim); pulls RIGHT, "
              "lower fl/rl. Nudge by ~0.02-0.03 and re-test.")
        return

    left = (counts["fl"] + counts["rl"]) / 2.0
    right = (counts["fr"] + counts["rr"]) / 2.0
    if left < right:
        print(f"\nLeft side traveled less ({left:.0f} vs {right:.0f}) -> robot "
              f"veers LEFT.")
    elif right < left:
        print(f"\nRight side traveled less ({right:.0f} vs {left:.0f}) -> robot "
              f"veers RIGHT.")
    else:
        print("\nLeft and right travel match -- already tracking straight.")

    # Normalize to the slowest wheel so trims stay <= 1.0 (never push duty past
    # 100%). Compose with current trims since the measured travel already
    # reflects them -- this makes repeated runs converge.
    slowest = min(counts.values())
    print("\nSuggested config.yml vehicle.motor_trim "
          "(current x slowest/this-wheel):")
    print("  motor_trim:")
    for w in WHEELS:
        suggested = kin.motor_trim[w] * (slowest / counts[w])
        print(f"    {w}: {suggested:.3f}")
    print("\nApply, then run `trim` again to refine (it composes with the "
          "values now in config.yml). Trim corrects average drift only; "
          "open-loop speed still varies with battery/load.")


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port (default %(default)s)")
    p.add_argument("--config", default=None, help="path to config.yml")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("limits", help="print theoretical velocity envelope")
    sub.add_parser("encoders", help="live encoder monitor")

    s = sub.add_parser("spin", help="spin each motor one at a time")
    s.add_argument("--percent", type=int, default=30)
    s.add_argument("--secs", type=float, default=2.0)

    c = sub.add_parser("cpr", help="measure counts per wheel revolution")
    c.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4])
    c.add_argument("--turns", type=float, default=10.0)

    d = sub.add_parser("drive", help="timed body-velocity drive test")
    d.add_argument("--vx", type=float, default=0.0)
    d.add_argument("--vy", type=float, default=0.0)
    d.add_argument("--wz", type=float, default=0.0)
    d.add_argument("--secs", type=float, default=5.0)
    d.add_argument("--cpr", type=float, default=None,
                   help="counts-per-rev (from `cpr`) to compute measured speed")

    t = sub.add_parser("trim", help="drive straight, suggest per-wheel motor_trim")
    t.add_argument("--vx", type=float, default=0.2)
    t.add_argument("--secs", type=float, default=4.0)
    t.add_argument("--runs", type=int, default=3,
                   help="number of drives to average (default %(default)s); "
                        "direction alternates to stay in place")
    t.add_argument("--pause", type=float, default=1.0,
                   help="seconds to settle between runs (default %(default)s)")
    return p


def main():
    args = build_parser().parse_args()
    # config defaults to the kinematics module's own default if not given
    if args.config is None:
        from lib.kinematics import DEFAULT_CONFIG_PATH
        args.config = DEFAULT_CONFIG_PATH

    handlers = {
        "limits": cmd_limits,
        "encoders": cmd_encoders,
        "spin": cmd_spin,
        "cpr": cmd_cpr,
        "drive": cmd_drive,
        "trim": cmd_trim,
    }
    handlers[args.cmd](args)


if __name__ == "__main__":
    main()
