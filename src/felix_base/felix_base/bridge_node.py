#!/usr/bin/env python3
"""
Base driver node for the custom Felix mecanum chassis.

Owns the single serial connection to the ROSMASTER board and is therefore the one
place that does everything board-related (you cannot open /dev/myserial twice):

  * subscribes /cmd_vel -> open-loop mecanum motor commands (set_motor)
  * publishes /odom + the odom->base_link TF from the wheel encoders
  * publishes /imu/data from the board IMU

Odometry is open-loop dead reckoning and drifts (heading worst -- wheel slip is
unobserved). The IMU is published alongside so a robot_localization EKF can fuse
gyro yaw on top of the wheel odometry later; this node intentionally publishes
the two raw streams and does no fusion itself.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from felix_base.rosmaster import Rosmaster
from felix_base.kinematics import MecanumKinematics
from felix_base.odometry import MecanumOdometry


def quaternion_from_yaw(yaw):
    """Planar (roll=pitch=0) yaw -> geometry_msgs/Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_from_rpy(roll, pitch, yaw):
    """ZYX roll/pitch/yaw (radians) -> geometry_msgs/Quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class RosmasterBridgeNode(Node):
    def __init__(self):
        super().__init__('rosmaster_bridge_node')

        # Declare parameters for dynamic port configuration
        self.declare_parameter('port', '/dev/myserial')
        serial_port = self.get_parameter('port').get_parameter_value().string_value

        # Custom mecanum kinematics (geometry/limits from config.yml). This
        # chassis is NOT a stock Yahboom frame, so the firmware's set_car_motion
        # geometry does not apply -- we compute per-wheel commands ourselves and
        # drive them open-loop via set_motor().
        self.declare_parameter('config', '')
        config_path = self.get_parameter('config').get_parameter_value().string_value
        self.kin = MecanumKinematics(config_path) if config_path else MecanumKinematics()
        self.get_logger().info(
            f"Loaded mecanum kinematics: v_max={self.kin.v_max:.2f} m/s, "
            f"wz_max={self.kin.wz_max:.2f} rad/s")

        # Odometry / IMU parameters.
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('sensor_rate', 50.0)        # Hz, encoder+IMU poll
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_imu = self.get_parameter('publish_imu').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        sensor_rate = float(self.get_parameter('sensor_rate').value)

        # Initialize Yahboom Hardware Serial Client
        try:
            self.bot = Rosmaster(com=serial_port)
            self.get_logger().info(f"Successfully linked to ROSMASTER Board on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open hardware board: {str(e)}")
            raise e

        # The background receive thread MUST be running before get_motor_encoder()
        # / get_*_data() return live values (vendor requirement).
        self.bot.create_receive_threading()

        # Subscribe to standard /cmd_vel geometric velocity commands
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- /cmd_vel watchdog ---------------------------------------------
        # set_motor() latches the last command indefinitely, so a crashed Nav2
        # planner, a hung script, or a dropped DDS link would otherwise leave the
        # robot driving forever. If no command arrives within cmd_vel_timeout
        # seconds, stop the motors. 0 disables. Armed only after the first
        # command (idle-at-startup is not a fault); re-arms when commands resume.
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self._last_cmd_time = None
        self._cmd_stopped = False
        self._last_cmd_moving = False
        if self.cmd_vel_timeout > 0.0:
            self.watchdog_timer = self.create_timer(0.1, self._cmd_watchdog)
            self.get_logger().info(
                f"/cmd_vel watchdog: stop motors after {self.cmd_vel_timeout:.2f}s of silence")

        # Odometry can only run if the encoders are calibrated (counts_per_rev).
        if self.publish_odom and not self.kin.counts_per_rev:
            self.get_logger().warning(
                "counts_per_rev is 0 in config.yml -- disabling odometry. Run "
                "`calibrate cpr` and set it to enable /odom.")
            self.publish_odom = False

        self.odom = MecanumOdometry(self.kin)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) if self.publish_odom else None
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10) if self.publish_imu else None
        self.tf_broadcaster = TransformBroadcaster(self) if (self.publish_odom and self.publish_tf) else None

        # Encoder/clock state for delta integration; seeded on the first tick.
        self._last_enc = None
        self._last_time = None

        if self.publish_odom or self.publish_imu:
            self.sensor_timer = self.create_timer(1.0 / sensor_rate, self._on_sensor_timer)
            self.get_logger().info(
                f"Publishing {'odom ' if self.publish_odom else ''}"
                f"{'imu ' if self.publish_imu else ''}at {sensor_rate:.0f} Hz")

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert a body-frame velocity command (m/s, rad/s) into per-wheel motor
        commands using our custom mecanum kinematics, then drive the four motors
        directly via set_motor() (percent duty, open loop).

        Every command is first clamped to the configured envelope and stripped of
        non-finite values (clamp_body) -- the single safety boundary shared by
        teleop and any future Nav2/SLAM planner. Magnitudes beyond the chassis
        limits are then additionally scaled down uniformly inside body_to_motor().
        """
        # x = forward/back (m/s), y = lateral strafe (m/s), z = yaw rate (rad/s)
        vx, vy, wz = self.kin.clamp_body(msg.linear.x, msg.linear.y, msg.angular.z)

        s1, s2, s3, s4 = self.kin.body_to_motor(vx, vy, wz)
        self.bot.set_motor(s1, s2, s3, s4)

        # Pet the watchdog and record whether this command was actually driving
        # (so the failsafe only warns when motion was interrupted, not on a
        # normal idle stop where teleop sends a single zero then goes quiet).
        self._last_cmd_time = self.get_clock().now()
        self._cmd_stopped = False
        self._last_cmd_moving = (vx != 0.0 or vy != 0.0 or wz != 0.0)

    def _cmd_watchdog(self):
        """Failsafe: stop the motors if /cmd_vel has gone silent past the timeout.

        Runs at 10 Hz. No-op until the first command (idle startup is not a
        fault) and once already stopped (avoids re-spamming set_motor). The
        warning fires only if the robot was moving when commands stopped -- the
        real "planner died mid-drive" case -- so a normal stop-and-go teleop
        session stays quiet."""
        if self._last_cmd_time is None or self._cmd_stopped:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            self.bot.set_motor(0, 0, 0, 0)
            self._cmd_stopped = True
            if self._last_cmd_moving:
                self.get_logger().warning(
                    f"/cmd_vel silent for {elapsed:.2f}s (> {self.cmd_vel_timeout:.2f}s) "
                    "while moving -- motors stopped (failsafe).")

    def _on_sensor_timer(self):
        now = self.get_clock().now()
        if self.publish_odom:
            self._update_odometry(now)
        if self.publish_imu:
            self._publish_imu(now)

    def _update_odometry(self, now):
        enc = self.bot.get_motor_encoder()             # (m1, m2, m3, m4), s1..s4 order

        # Seed baseline on the first tick (no delta yet).
        if self._last_enc is None:
            self._last_enc = enc
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        delta = tuple(enc[i] - self._last_enc[i] for i in range(4))
        self._last_enc = enc
        self._last_time = now

        wheel_rads = self.kin.encoder_deltas_to_wheel_rads(delta)
        if wheel_rads is None:
            return
        vx, vy, wz = self.odom.update(wheel_rads, dt)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.odom.x
        odom_msg.pose.pose.position.y = self.odom.y
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.odom.theta)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        # Placeholder diagonal covariances: x most trusted, strafe (y) less, and
        # wheel-derived yaw least (slip is unobserved). Unused planar-out dims
        # (z, roll, pitch) are flagged large. Tune once the EKF is in.
        odom_msg.pose.covariance = self._diag6(0.01, 0.05, 1e6, 1e6, 1e6, 0.2)
        odom_msg.twist.covariance = self._diag6(0.01, 0.05, 1e6, 1e6, 1e6, 0.2)
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom.x
            tf.transform.translation.y = self.odom.y
            tf.transform.rotation = quaternion_from_yaw(self.odom.theta)
            self.tf_broadcaster.sendTransform(tf)

    def _publish_imu(self, now):
        # Vendor getters return processed values: gyro in rad/s and accel in m/s^2
        # on the MPU9250 firmware path (see Rosmaster receive thread). If this
        # board is an ICM20948 the raw ratios differ -- sanity-check by rotating
        # the robot and watching angular_velocity.z. Attitude is the board's fused
        # roll/pitch/yaw in radians.
        gx, gy, gz = self.bot.get_gyroscope_data()
        ax, ay, az = self.bot.get_accelerometer_data()
        roll, pitch, yaw = self.bot.get_imu_attitude_data(ToAngle=False)

        # The board reports the gravity vector (down-positive): level-at-rest reads
        # az ~= -9.66, but REP-145 wants specific force (up-positive, +9.81). Negate
        # the accel vector so a stationary, level IMU reads +z, consistent with the
        # z-up frame the gyro already uses (confirmed: CCW -> +angular_velocity.z).
        ax, ay, az = -ax, -ay, -az

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame
        imu.orientation = quaternion_from_rpy(roll, pitch, yaw)
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        # Gyro Z is inverted vs REP-103 on this board: a physical CCW (left) spin
        # reads NEGATIVE raw (verified live -- holding teleop 'j', which drives a
        # confirmed CCW spin, gave angular_velocity.z ~ -0.88). REP-103 wants
        # CCW = +z, so negate. This is the only IMU axis the EKF fuses (vyaw), so
        # it sets the heading sense. (gx/gy are unused/unverified; the full
        # IMU->base_link rotation belongs in felix_description's imu_link later.)
        imu.angular_velocity.z = -gz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.orientation_covariance = self._diag3(0.05, 0.05, 0.1)
        imu.angular_velocity_covariance = self._diag3(0.01, 0.01, 0.01)
        imu.linear_acceleration_covariance = self._diag3(0.1, 0.1, 0.1)
        self.imu_pub.publish(imu)

    @staticmethod
    def _diag6(a, b, c, d, e, f):
        m = [0.0] * 36
        m[0], m[7], m[14], m[21], m[28], m[35] = a, b, c, d, e, f
        return m

    @staticmethod
    def _diag3(a, b, c):
        return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]

    def destroy_node(self):
        # Safety Protocol: Stop moving the platform instantly upon system shutdown.
        # Use print(), not get_logger(): on Ctrl-C the rclpy context is already
        # torn down, so rosout publishing fails with "publisher's context is invalid".
        print("Stopping robot platform safely...")
        self.bot.set_motor(0, 0, 0, 0)
        del self.bot
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
