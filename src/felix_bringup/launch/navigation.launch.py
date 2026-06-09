"""One-shot launch for AUTONOMOUS NAVIGATION on a saved map.

The full autonomy stack under a single ros2 launch (one Ctrl-C stops all of it):
it reuses localization.launch.py (base + EKF + description + RPLIDAR + map_server +
AMCL + camera + foxglove) and adds the felix_nav nav2 servers on top.

  localization.launch.py  -> everything localization (see that file)
  felix_nav/navigation    -> planner + MPPI controller + behaviors + BT navigator

Send a goal from the Foxglove/rviz "Nav2 Goal" tool; the robot plans a path and
drives there (strafing as needed -- it's holonomic). First seed AMCL with a
"2D Pose Estimate" so it knows where it starts.

    ros2 launch felix_bringup navigation.launch.py
    ros2 launch felix_bringup navigation.launch.py map:=/felix-ai-ros/maps/other.yaml
    ros2 launch felix_bringup navigation.launch.py camera:=false

Args (in addition to localization.launch.py's port/map/camera/foxglove/...):
  navigation   run the nav2 navigation servers (default true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, condition=None, args=None):
    path = PathJoinSubstitution([FindPackageShare(pkg)] + rel)
    kw = {}
    if condition is not None:
        kw["condition"] = condition
    if args is not None:
        kw["launch_arguments"] = args.items()
    return IncludeLaunchDescription(PythonLaunchDescriptionSource([path]), **kw)


def generate_launch_description():
    port = LaunchConfiguration("port")
    map_yaml = LaunchConfiguration("map")
    camera = LaunchConfiguration("camera")
    foxglove = LaunchConfiguration("foxglove")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    navigation = LaunchConfiguration("navigation")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/myserial"),
        DeclareLaunchArgument("map",
                              default_value="/felix-ai-ros/maps/felix_map.yaml"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("navigation", default_value="true"),

        # Full localization stack (base + EKF + description + lidar + map_server
        # + AMCL + camera + foxglove). Reuses the one-shot localization launch.
        _inc("felix_bringup", ["launch", "localization.launch.py"],
             args={
                 "port": port,
                 "map": map_yaml,
                 "camera": camera,
                 "foxglove": foxglove,
                 "serial_baudrate": serial_baudrate,
             }),

        # nav2 navigation servers on top.
        _inc("felix_nav", ["launch", "navigation.launch.py"],
             condition=IfCondition(navigation)),
    ])
