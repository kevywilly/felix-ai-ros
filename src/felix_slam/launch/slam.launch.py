"""Felix 2D SLAM: RPLIDAR + slam_toolbox (online async mapping).

Run this ALONGSIDE felix_bringup, which must already be providing:
  * odom -> base_link  (the felix_localization EKF)
  * base_link -> laser (felix_description / robot_state_publisher)

This launch adds the lidar (/scan) and slam_toolbox, which publishes map -> odom
and the /map occupancy grid. Full TF chain:

    map -> odom (slam_toolbox) -> base_link (EKF) -> laser (URDF)

    Terminal 1:  ros2 launch felix_bringup felix.launch.py
    Terminal 2:  ros2 launch felix_slam slam.launch.py
    Terminal 3:  ros2 run felix_base teleop        # drive to build the map

Resuming / extending an existing map:
  This ALWAYS starts a fresh map -- there is NO startup launch arg for it. The
  mapping node (async_slam_toolbox_node) ignores the `map_file_name` /
  `map_start_pose` / `map_start_at_dock` params; slam_toolbox only auto-loads
  those in LOCALIZATION mode. To keep mapping on top of a saved pose-graph you
  must call the deserialize_map service AFTER this is up -- use `./resume_map.sh`
  (robot parked where the original map began). See resume_map.sh / README.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    pkg = FindPackageShare("felix_slam")
    rplidar_launch = PathJoinSubstitution([pkg, "launch", "rplidar.launch.py"])
    slam_params = PathJoinSubstitution([pkg, "config", "slam_toolbox.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLIDAR serial device."),
        DeclareLaunchArgument("serial_baudrate", default_value="115200",
                              description="Baud: 115200 (A1) or 256000 (A2/A3/S1)."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch]),
            launch_arguments={
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
            }.items(),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
    ])
