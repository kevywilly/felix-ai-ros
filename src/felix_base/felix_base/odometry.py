#!/usr/bin/env python3
"""
Wheel-encoder odometry for the custom Felix mecanum chassis.

Integrates per-wheel angular displacements (from the encoders) into a planar
pose (x, y, theta) in the `odom` frame, using the chassis forward kinematics in
MecanumKinematics. This is dead reckoning: it drifts, and on an open-loop mecanum
the heading (theta) is the least trustworthy component because lateral/rotational
wheel slip is not observed. Fuse the IMU yaw on top (robot_localization EKF) for
a usable `odom -> base_link` once both streams are verified.

Pure Python, no ROS dependency, so the integration can be unit-tested directly.
"""
import math

from felix_base.kinematics import WHEELS


class MecanumOdometry:
    def __init__(self, kinematics):
        self.kin = kinematics
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta

    def update(self, wheel_rads, dt):
        """Advance the pose by one step.

        wheel_rads: dict keyed by WHEELS, each the wheel's angular displacement
                    (radians) over this step (see
                    MecanumKinematics.encoder_deltas_to_wheel_rads).
        dt:         elapsed seconds for this step (> 0).

        Returns the body-frame twist (vx, vy, wz) for this step so the caller can
        populate the Odometry message's twist field.
        """
        if dt <= 0.0:
            return 0.0, 0.0, 0.0

        # Wheel displacement (rad) -> wheel angular velocity (rad/s) -> body twist.
        omegas = {w: wheel_rads[w] / dt for w in WHEELS}
        vx, vy, wz = self.kin.wheels_to_body(omegas)

        # Integrate in the odom frame. Use the mid-step heading so a combined
        # translate+rotate step is integrated more accurately than naive Euler.
        dtheta = wz * dt
        dx_b = vx * dt
        dy_b = vy * dt
        theta_mid = self.theta + 0.5 * dtheta
        cos_m, sin_m = math.cos(theta_mid), math.sin(theta_mid)

        self.x += dx_b * cos_m - dy_b * sin_m
        self.y += dx_b * sin_m + dy_b * cos_m
        self.theta = self._normalize(self.theta + dtheta)

        return vx, vy, wz

    @staticmethod
    def _normalize(angle):
        """Wrap to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
