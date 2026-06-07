#!/usr/bin/env python3
"""
Mecanum kinematics for the custom Felix chassis.

This is the single source of truth for turning a body-frame velocity command
(vx, vy, wz) into the four motor commands the ROSMASTER board expects via
Rosmaster.set_motor(s1, s2, s3, s4), where each value is a percent duty in
[-100, 100] (open loop -- the firmware PID is NOT used on this path).

All geometry/motor parameters come from config.yml so there is exactly one
place to edit them. Conventions follow ROS REP-103:
    vx > 0  : forward
    vy > 0  : left (strafe)
    wz > 0  : counter-clockwise (turn left)
"""
import math
import os

import yaml

# Standard mecanum wheel order used throughout this module: front-left,
# front-right, rear-left, rear-right.
WHEELS = ("fl", "fr", "rl", "rr")

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                   "config.yml")


class MecanumKinematics:
    def __init__(self, config_path=DEFAULT_CONFIG_PATH):
        with open(config_path, "r") as fh:
            cfg = yaml.safe_load(fh)
        v = cfg["vehicle"]

        # --- Geometry (metres) ---
        self.wheel_radius = float(v["wheel_radius"])
        # lx = half the wheelbase (front<->rear), ly = half the track (left<->right)
        self.lx = float(v["wheel_base"]) / 2.0
        self.ly = float(v["track_width"]) / 2.0
        self.lxy = self.lx + self.ly

        # --- Motor limits ---
        self.max_rpm = float(v["max_rpm"])
        self.min_rpm = float(v.get("min_rpm", 0.0))
        self.gear_ratio = float(v.get("gear_ratio", 1.0))

        # Encoder counts per wheel revolution (post-gearbox), measured via
        # `calibrate.py cpr`. 0/None means encoders are not calibrated.
        self.counts_per_rev = float(v.get("counts_per_rev", 0) or 0)
        self.wheel_circumference = 2.0 * math.pi * float(v["wheel_radius"])

        # Open-loop correction: real speed = commanded / velocity_scale, so we
        # pre-multiply commands by this factor to make actual match commanded.
        # Measured via `calibrate.py drive` (scale = commanded / measured). 1.0
        # = no correction.
        self.velocity_scale = float(v.get("velocity_scale", 1.0) or 1.0)

        # Wheel angular-speed limits (rad/s). max_rpm is the OUTPUT (post-gearbox)
        # speed, i.e. the wheel speed, so no gear_ratio division here.
        self.w_max = self.max_rpm / 60.0 * 2.0 * math.pi
        self.w_min = self.min_rpm / 60.0 * 2.0 * math.pi

        # --- Robot velocity limits (each in isolation) ---
        self.v_max = self.wheel_radius * self.w_max          # max vx (m/s)
        self.vx_max = self.v_max
        self.vy_max = self.v_max                              # strafe (real value is lower; see note in calibrate.py)
        self.wz_max = self.v_max / self.lxy                   # max yaw rate (rad/s)

        # --- Open-loop percent mapping ---
        # Below min_rpm the gearmotor stalls, so any non-zero wheel command is
        # lifted to at least this duty (deadband compensation). Calibration may
        # refine the value.
        self.deadband_pct = (self.min_rpm / self.max_rpm * 100.0) if self.max_rpm else 0.0

        # --- Wheel order / sign mapping to set_motor(s1, s2, s3, s4) ---
        # Which physical motor index (1..4) each logical wheel maps to, and the
        # sign that makes a positive command drive that wheel forward. Defaults
        # are a guess; calibrate.py tells you the correct values to put in
        # config.yml under vehicle.motor_map / vehicle.motor_sign.
        mapping = v.get("motor_map", {"fl": 1, "fr": 2, "rl": 3, "rr": 4})
        signs = v.get("motor_sign", {"fl": 1, "fr": 1, "rl": 1, "rr": 1})
        self.motor_map = {w: int(mapping[w]) for w in WHEELS}
        self.motor_sign = {w: int(signs[w]) for w in WHEELS}

        # Per-wheel open-loop gain trim (dimensionless, ~1.0). Open loop has no
        # PID, so nominally-equal motors run at slightly different real speeds,
        # which makes a straight-forward command veer. Lower the trim on the
        # faster side (or raise it on the slower side) until the robot tracks
        # straight. Default 1.0 = no trim. See note in config.yml.
        trims = v.get("motor_trim", {})
        self.motor_trim = {w: float(trims.get(w, 1.0)) for w in WHEELS}

    # ------------------------------------------------------------------ #
    # Kinematics
    # ------------------------------------------------------------------ #
    def body_to_wheels(self, vx, vy, wz):
        """Body velocity -> wheel angular velocities (rad/s), order WHEELS."""
        r = self.wheel_radius
        k = self.lxy
        return {
            "fl": (vx - vy - k * wz) / r,
            "fr": (vx + vy + k * wz) / r,
            "rl": (vx + vy - k * wz) / r,
            "rr": (vx - vy + k * wz) / r,
        }

    def saturate(self, wheels):
        """Scale all wheels down uniformly if any exceeds w_max (preserves direction)."""
        peak = max((abs(w) for w in wheels.values()), default=0.0)
        if peak > self.w_max and peak > 0.0:
            scale = self.w_max / peak
            return {k: w * scale for k, w in wheels.items()}, scale
        return dict(wheels), 1.0

    def wheel_to_percent(self, w):
        """One wheel angular velocity (rad/s) -> set_motor percent [-100, 100]."""
        if abs(w) < 1e-6:
            return 0.0
        frac = min(abs(w) / self.w_max, 1.0)              # 0..1 of max speed
        # Map [0,1] onto [deadband, 100] so even tiny commands overcome stall.
        pct = self.deadband_pct + frac * (100.0 - self.deadband_pct)
        return math.copysign(pct, w)

    def body_to_motor(self, vx, vy, wz):
        """
        Full pipeline: body velocity -> ordered, signed integer motor commands
        (s1, s2, s3, s4) ready for Rosmaster.set_motor().
        """
        s = self.velocity_scale
        wheels, _ = self.saturate(self.body_to_wheels(vx * s, vy * s, wz * s))
        pct = {w: self.wheel_to_percent(wheels[w]) for w in WHEELS}
        motors = [0, 0, 0, 0]
        for w in WHEELS:
            idx = self.motor_map[w] - 1                   # set_motor is 1-indexed
            duty = self.motor_sign[w] * self.motor_trim[w] * pct[w]
            duty = max(-100.0, min(100.0, duty))          # trim may push past 100
            motors[idx] = int(round(duty))
        return tuple(motors)

    def counts_to_distance(self, counts):
        """Encoder counts -> wheel-path distance in metres (needs counts_per_rev)."""
        if not self.counts_per_rev:
            return None
        return counts / self.counts_per_rev * self.wheel_circumference

    def describe_limits(self):
        """Human-readable summary of the derived velocity envelope."""
        return (
            f"Wheel radius        : {self.wheel_radius:.4f} m\n"
            f"Half wheelbase  lx  : {self.lx:.4f} m\n"
            f"Half track      ly  : {self.ly:.4f} m\n"
            f"Max wheel speed     : {self.max_rpm:.1f} rpm  = {self.w_max:.3f} rad/s\n"
            f"Min wheel speed     : {self.min_rpm:.1f} rpm  = {self.w_min:.3f} rad/s\n"
            f"--> Max linear vx/vy: {self.v_max:.3f} m/s\n"
            f"--> Max yaw rate wz : {self.wz_max:.3f} rad/s\n"
            f"--> Min linear speed: {self.wheel_radius * self.w_min:.3f} m/s (below this a wheel may stall)\n"
            f"Stall deadband duty : {self.deadband_pct:.1f} %"
        )


if __name__ == "__main__":
    print(MecanumKinematics().describe_limits())
