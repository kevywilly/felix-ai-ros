#!/usr/bin/env python3
"""
Keyboard teleop node.

Reads single keystrokes from the terminal and publishes geometry_msgs/Twist
messages on /cmd_vel. The rosmaster_bridge_node subscribes to /cmd_vel and
forwards the velocities to the Yahboom ROSMASTER board via set_car_motion().

Run from the repository root (same as rosmaster_bridge_node) so the ROS graph
lines up:

    python3 main.py
"""
import sys
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from lib.kinematics import MecanumKinematics


MSG = """
Reading from the keyboard and publishing to /cmd_vel
----------------------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i / , : forward / backward
j / l : rotate left / right
u o m . : combined drive + turn
k or space : stop

Holonomic strafe (hold Shift):
   U    I    O
   J    K    L
   M    <    >

Speed control:
   q / z : increase / decrease all speeds by 10%
   w / x : increase / decrease linear speed by 10%
   e / c : increase / decrease angular speed by 10%

CTRL-C to quit
"""

# Each binding maps a key to (x, y, theta) direction multipliers.
MOVE_BINDINGS = {
    'i': (1, 0, 0),
    'o': (1, 0, -1),
    'j': (0, 0, 1),
    'l': (0, 0, -1),
    'u': (1, 0, 1),
    ',': (-1, 0, 0),
    '.': (-1, 0, 1),
    'm': (-1, 0, -1),
    'I': (1, 0, 0),
    'O': (1, -1, 0),
    'J': (0, 1, 0),
    'L': (0, -1, 0),
    'U': (1, 1, 0),
    '<': (-1, 0, 0),
    '>': (-1, -1, 0),
    'M': (-1, 1, 0),
}

# Each binding scales (linear, angular) speed.
SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

STOP_KEYS = ('k', 'K', ' ')


def get_key(settings, timeout=0.1):
    """Read a single keystroke (non-blocking) from stdin in raw mode."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Velocity limits come from the chassis config (config.yml) so teleop
        # never commands past what the motors/geometry can actually do.
        kin = MecanumKinematics()
        self.linear_limit = kin.v_max
        self.angular_limit = kin.wz_max

        # Tunable starting (cruise) speeds, kept within the limits above.
        self.declare_parameter('linear_speed', min(0.2, self.linear_limit))
        self.declare_parameter('angular_speed', min(1.0, self.angular_limit))
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # How long a movement command stays active after the last keystroke.
        # Holding a key sends one char, then the OS waits out the keyboard
        # "typematic delay" (~0.3-0.5s) before auto-repeat begins. If we stopped
        # on every idle tick the robot would stutter (go/stop/go) on that gap,
        # so we keep the last command alive for this window. It is also the
        # coast time after you release a key, so keep it just above the gap.
        self.declare_parameter('key_timeout', 0.6)
        self.key_timeout = self.get_parameter('key_timeout').value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def publish(self, x, y, theta):
        msg = Twist()
        msg.linear.x = self.clamp(x * self.linear_speed, self.linear_limit)
        msg.linear.y = self.clamp(y * self.linear_speed, self.linear_limit)
        msg.angular.z = self.clamp(theta * self.angular_speed, self.angular_limit)
        self.publisher.publish(msg)

    def stop(self):
        self.publisher.publish(Twist())

    def speed_status(self):
        return (f"currently:\tlinear {self.linear_speed:.2f} m/s"
                f"\tangular {self.angular_speed:.2f} rad/s")


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(MSG)
    print(node.speed_status())

    last_move = None        # last movement (x, y, theta) being held
    last_move_time = 0.0    # monotonic time of the last movement keystroke

    try:
        while rclpy.ok():
            key = get_key(settings)
            now = time.monotonic()

            if key in MOVE_BINDINGS:
                last_move = MOVE_BINDINGS[key]
                last_move_time = now
                node.publish(*last_move)
            elif key in SPEED_BINDINGS:
                lin_scale, ang_scale = SPEED_BINDINGS[key]
                node.linear_speed *= lin_scale
                node.angular_speed *= ang_scale
                print(node.speed_status())
            elif key in STOP_KEYS:
                last_move = None
                node.stop()
            elif key == '\x03':  # CTRL-C
                break
            elif key == '' and last_move is not None \
                    and (now - last_move_time) < node.key_timeout:
                # Idle tick within the hold window: this is the typematic gap
                # between auto-repeats, not a release. Keep the command alive so
                # the robot does not stutter (go/stop/go) at the start of motion.
                node.publish(*last_move)
            else:
                # Hold window elapsed (key released) or an unmapped key: stop so
                # the robot does not keep coasting on the last command.
                last_move = None
                node.stop()

            rclpy.spin_once(node, timeout_sec=0)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Teleop error: {exc}")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
