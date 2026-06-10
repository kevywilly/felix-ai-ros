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
  By default this starts a FRESH map. To instead load a previously serialized
  pose-graph and keep mapping on top of it, pass `map_file_name` (the basename of
  the .posegraph/.data pair, NO extension), saved by save_map.sh:

      ros2 launch felix_slam slam.launch.py \
          map_file_name:=/felix-ai-ros/maps/felix_map

  slam_toolbox does NOT relocalize globally when it loads a graph -- it stitches
  new scans relative to a starting pose. With `map_start_at_dock:=true` (default)
  the robot's current spot is taken as the map origin, so START IT WHERE YOU FIRST
  BEGAN MAPPING. If you're elsewhere, pass a known pose instead, e.g.
  `map_start_pose:=[1.0, 2.0, 0.0]` (x, y, theta in the map frame); a wrong/absent
  pose mis-aligns the old graph with new scans and corrupts the map.
  Note: a .pgm/.yaml occupancy grid CANNOT be resumed -- only the pose-graph can.

Args:
  serial_port       lidar device (default /dev/rplidar)
  serial_baudrate   115200 (A1) | 256000 (A2/A3/S1)
  map_file_name     pose-graph basename to resume from (default "" = fresh map)
  map_start_at_dock take current pose as map origin when resuming (default true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    map_file_name = LaunchConfiguration("map_file_name")
    map_start_at_dock = LaunchConfiguration("map_start_at_dock")

    pkg = FindPackageShare("felix_slam")
    rplidar_launch = PathJoinSubstitution([pkg, "launch", "rplidar.launch.py"])
    slam_params = PathJoinSubstitution([pkg, "config", "slam_toolbox.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLIDAR serial device."),
        DeclareLaunchArgument("serial_baudrate", default_value="115200",
                              description="Baud: 115200 (A1) or 256000 (A2/A3/S1)."),
        DeclareLaunchArgument("map_file_name", default_value="",
                              description="Pose-graph basename to resume from "
                                          "(no extension). Empty = fresh map."),
        DeclareLaunchArgument("map_start_at_dock", default_value="true",
                              description="When resuming, take the robot's current "
                                          "pose as the map origin."),

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
            # Launch-arg overrides come AFTER the yaml so they win. An empty
            # map_file_name leaves slam_toolbox in its default fresh-map mode.
            parameters=[slam_params, {
                "map_file_name": map_file_name,
                "map_start_at_dock": ParameterValue(map_start_at_dock,
                                                    value_type=bool),
            }],
        ),
    ])
