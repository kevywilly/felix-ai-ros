"""Felix localization on a SAVED map: RPLIDAR + nav2 map_server + AMCL.

The "run mode" counterpart to slam.launch.py. Instead of building a map,
map_server serves a static occupancy grid (the .pgm/.yaml you saved) and AMCL
localizes against it, publishing map -> odom on top of the EKF's odom -> base_link.

Run this ALONGSIDE felix_bringup (base + EKF + description), and do NOT run
slam.launch.py at the same time -- both publish map -> odom and would fight.

    Terminal 1:  ros2 launch felix_bringup felix.launch.py
    Terminal 2:  ros2 launch felix_slam localization.launch.py map:=/felix-ai-ros/maps/felix_map.yaml
    Terminal 3:  ros2 run felix_base teleop          # drive; watch the pose track

After launch, seed AMCL with the robot's real start pose via a "2D Pose Estimate"
in Foxglove/rviz (publishes /initialpose) unless it actually starts at the map
origin. map_server + AMCL are lifecycle nodes, brought up by lifecycle_manager.

Args:
  map              occupancy-grid yaml (default /felix-ai-ros/maps/felix_map.yaml)
  use_rplidar      also launch the lidar here (default true; false if already up)
  serial_port      lidar device (default /dev/rplidar)
  serial_baudrate  115200 (A1) | 256000 (A2/A3/S1)
  scan_mode        Standard | Sensitivity | Boost
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    use_rplidar = LaunchConfiguration("use_rplidar")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    scan_mode = LaunchConfiguration("scan_mode")

    pkg = FindPackageShare("felix_slam")
    rplidar_launch = PathJoinSubstitution([pkg, "launch", "rplidar.launch.py"])
    amcl_params = PathJoinSubstitution([pkg, "config", "amcl.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="/felix-ai-ros/maps/felix_map.yaml",
            description="Occupancy-grid map yaml to localize against."),
        DeclareLaunchArgument(
            "use_rplidar", default_value="true",
            description="Launch the RPLIDAR here too (false if already running)."),
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLIDAR serial device."),
        DeclareLaunchArgument("serial_baudrate", default_value="115200",
                              description="Baud: 115200 (A1) or 256000 (A2/A3/S1)."),
        DeclareLaunchArgument("scan_mode", default_value="Standard",
                              description="RPLIDAR scan mode."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch]),
            condition=IfCondition(use_rplidar),
            launch_arguments={
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "scan_mode": scan_mode,
            }.items(),
        ),

        # Serves the static /map occupancy grid from the saved yaml.
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "yaml_filename": map_yaml,
            }],
        ),

        # Localizes the robot on /map, publishing map -> odom.
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params],
        ),

        # map_server + amcl are lifecycle nodes; this configures+activates them.
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
    ])
