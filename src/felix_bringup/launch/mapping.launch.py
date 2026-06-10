"""One-shot launch for the full mapping/driving stack.

Composes everything you need to drive around and build a map, as children of a
single ros2 launch -- so ONE Ctrl-C cleanly stops all of it (no orphaned nodes):

  * felix.launch.py     base driver + EKF + robot description (TF)
  * slam.launch.py      RPLIDAR + slam_toolbox          (slam:=false to skip)
  * camera.launch.py    CSI camera -> compressed image   (camera:=false to skip)
  * foxglove_bridge     WebSocket for the Mac UI         (foxglove:=false to skip)

Drive from the Foxglove Teleop panel (publishes /cmd_vel) -- no teleop terminal
needed. (teleop_twist_keyboard still works in its own terminal if you prefer.)

    ros2 launch felix_bringup mapping.launch.py
    ros2 launch felix_bringup mapping.launch.py camera:=false        # no camera
    ros2 launch felix_bringup mapping.launch.py serial_baudrate:=256000
    # resume/extend a saved map (start the robot where you first began mapping):
    ros2 launch felix_bringup mapping.launch.py \
        map_file_name:=/felix-ai-ros/maps/felix_map

Args:
  port             ROSMASTER serial port (default /dev/myserial)
  use_ekf          run the EKF (default true)
  slam             run RPLIDAR + slam_toolbox (default true)
  camera           run the CSI camera (default true)
  foxglove         run foxglove_bridge (default true)
  foxglove_port    foxglove_bridge WebSocket port (default 8765)
  serial_baudrate  RPLIDAR baud, 115200 (A1) or 256000 (A2/S1) (default 115200)
  map_file_name    pose-graph basename to RESUME/extend an existing map from
                   (no extension; default "" = fresh map). See slam.launch.py.
  map_start_at_dock  take current pose as map origin when resuming (default true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, condition=None, args=None, xml=False):
    path = PathJoinSubstitution([FindPackageShare(pkg)] + rel)
    src = AnyLaunchDescriptionSource(path) if xml \
        else PythonLaunchDescriptionSource([path])
    kw = {}
    if condition is not None:
        kw["condition"] = condition
    if args is not None:
        kw["launch_arguments"] = args.items()
    return IncludeLaunchDescription(src, **kw)


def generate_launch_description():
    port = LaunchConfiguration("port")
    use_ekf = LaunchConfiguration("use_ekf")
    slam = LaunchConfiguration("slam")
    camera = LaunchConfiguration("camera")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    map_file_name = LaunchConfiguration("map_file_name")
    map_start_at_dock = LaunchConfiguration("map_start_at_dock")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/myserial"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("map_file_name", default_value=""),
        DeclareLaunchArgument("map_start_at_dock", default_value="true"),

        # Core: base driver + EKF + description (always on).
        _inc("felix_bringup", ["launch", "felix.launch.py"],
             args={"port": port, "use_ekf": use_ekf}),

        # SLAM: RPLIDAR + slam_toolbox. map_file_name (empty by default) resumes
        # an existing pose-graph instead of starting fresh.
        _inc("felix_slam", ["launch", "slam.launch.py"],
             condition=IfCondition(slam),
             args={"serial_baudrate": serial_baudrate,
                   "map_file_name": map_file_name,
                   "map_start_at_dock": map_start_at_dock}),

        # CSI camera (low-lag defaults).
        _inc("felix_camera", ["launch", "camera.launch.py"],
             condition=IfCondition(camera)),

        # Foxglove bridge (XML launch) for the Mac UI. Pass `port` explicitly:
        # the bridge's `port` arg is its WebSocket port (integer 8765), and
        # without this it would inherit our string `port` (/dev/myserial) from
        # the parent launch scope and abort on the type mismatch.
        _inc("foxglove_bridge", ["launch", "foxglove_bridge_launch.xml"],
             condition=IfCondition(foxglove), xml=True,
             args={"port": foxglove_port, "max_qos_depth": "400"}),
    ])
