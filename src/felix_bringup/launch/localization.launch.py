"""One-shot launch for driving on a SAVED map (localization, not mapping).

The run-mode twin of mapping.launch.py: identical stack, except it localizes
against a saved occupancy grid with nav2 map_server + AMCL instead of building a
new map with slam_toolbox. Everything runs as children of a single ros2 launch,
so ONE Ctrl-C cleanly stops all of it.

  * felix.launch.py            base driver + EKF + robot description (TF)
  * felix_slam/localization    RPLIDAR + map_server + AMCL    (localize:=false to skip)
  * camera.launch.py           CSI camera -> compressed image (camera:=false to skip)
  * felix_perception           YOLO detector + lidar fusion   (perception:=true to add)
  * foxglove_bridge            WebSocket for the Mac UI        (foxglove:=false to skip)

Drive from the Foxglove Teleop panel (publishes /cmd_vel) -- no teleop terminal.
AMCL starts at the map origin; if the robot does not start there, drop a "2D Pose
Estimate" in Foxglove (publishes /initialpose) to seed it, then drive to converge.

    ros2 launch felix_bringup localization.launch.py
    ros2 launch felix_bringup localization.launch.py map:=/felix-ai-ros/maps/other.yaml
    ros2 launch felix_bringup localization.launch.py camera:=false

Args:
  port             ROSMASTER serial port (default /dev/myserial)
  use_ekf          run the EKF (default true)
  localize         run RPLIDAR + map_server + AMCL (default true)
  map              occupancy-grid yaml (default /felix-ai-ros/maps/felix_map.yaml)
  camera           run the CSI camera (default true)
  perception       run felix_perception (YOLO + lidar fusion) (default false)
  foxglove         run foxglove_bridge (default true)
  foxglove_port    foxglove_bridge WebSocket port (default 8765)
  serial_baudrate  RPLIDAR baud, 256000 (A2 M12/A3/S1) or 115200 (A1) (default 256000)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
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
    localize = LaunchConfiguration("localize")
    map_yaml = LaunchConfiguration("map")
    camera = LaunchConfiguration("camera")
    perception = LaunchConfiguration("perception")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/myserial"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("localize", default_value="true"),
        DeclareLaunchArgument("map",
                              default_value="/felix-ai-ros/maps/felix_map.yaml"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("perception", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("serial_baudrate", default_value="256000"),

        # Core: base driver + EKF + description (always on).
        _inc("felix_bringup", ["launch", "felix.launch.py"],
             args={"port": port, "use_ekf": use_ekf}),

        # Localization: RPLIDAR + map_server + AMCL on the saved map.
        _inc("felix_slam", ["launch", "localization.launch.py"],
             condition=IfCondition(localize),
             args={"serial_baudrate": serial_baudrate, "map": map_yaml}),

        # CSI camera (low-lag defaults). DELAYED 5 s: the nvargus camera pipeline
        # init is CPU-heavy on the Jetson and, if it starts at t=0 alongside the
        # RPLIDAR driver, it starves the lidar's scan-start handshake past its
        # ~2 s SDK read timeout -> rplidar_node dies with SL_RESULT_OPERATION_TIMEOUT.
        # Letting the lidar establish its scan loop first avoids the race.
        TimerAction(period=5.0, actions=[
            _inc("felix_camera", ["launch", "camera.launch.py"],
                 condition=IfCondition(camera)),
        ]),

        # Perception: YOLO detector + lidar fusion (opt-in; needs the camera and,
        # for map placement, AMCL). The detector calibration matches the camera's
        # 640x360 bringup resolution.
        _inc("felix_perception", ["launch", "perception.launch.py"],
             condition=IfCondition(perception)),

        # Foxglove bridge (XML launch) for the Mac UI. Pass `port` explicitly:
        # the bridge's `port` arg is its WebSocket port (integer 8765), and
        # without this it would inherit our string `port` (/dev/myserial) from
        # the parent launch scope and abort on the type mismatch.
        _inc("foxglove_bridge", ["launch", "foxglove_bridge_launch.xml"],
             condition=IfCondition(foxglove), xml=True,
             args={"port": foxglove_port, "max_qos_depth": "400"}),
    ])
