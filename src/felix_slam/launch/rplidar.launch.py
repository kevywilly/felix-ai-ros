"""RPLIDAR driver: publish /scan in the 'laser' frame.

The lidar is on its own serial port (/dev/rplidar), separate from the ROSMASTER
board (/dev/myserial). frame_id MUST be 'laser' to match felix_description's URDF
(base_link -> laser), or slam_toolbox can't place the scans on the robot.

Args:
  serial_port      lidar device (default /dev/rplidar)
  serial_baudrate  115200 for RPLIDAR A1; A2/A3/S1 use 256000 -- change if no scan
  scan_mode        Standard | Sensitivity | Boost (driver/model dependent)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    scan_mode = LaunchConfiguration("scan_mode")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLIDAR serial device."),
        DeclareLaunchArgument("serial_baudrate", default_value="256000",
                              description="Baud: 256000 (A2 M12/A3/S1) or 115200 (A1)."),
        DeclareLaunchArgument("scan_mode", default_value="Standard",
                              description="RPLIDAR scan mode."),
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": "laser",          # must match the URDF laser frame
                "angle_compensate": True,
                "scan_mode": scan_mode,
                "inverted": False,
            }],
        ),
    ])
