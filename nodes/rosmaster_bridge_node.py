#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from lib.rosmaster import Rosmaster
from lib.kinematics import MecanumKinematics


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

        # Initialize Yahboom Hardware Serial Client
        try:
            self.bot = Rosmaster(com=serial_port)
            self.get_logger().info(f"Successfully linked to ROSMASTER Board on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open hardware board: {str(e)}")
            raise e

        # Subscribe to standard /cmd_vel geometric velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # QoS History Depth
        )
        
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert a body-frame velocity command (m/s, rad/s) into per-wheel motor
        commands using our custom mecanum kinematics, then drive the four motors
        directly via set_motor() (percent duty, open loop). Commands beyond the
        chassis limits are scaled down uniformly inside body_to_motor().
        """
        # x = forward/back (m/s), y = lateral strafe (m/s), z = yaw rate (rad/s)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        s1, s2, s3, s4 = self.kin.body_to_motor(vx, vy, wz)
        self.bot.set_motor(s1, s2, s3, s4)

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
