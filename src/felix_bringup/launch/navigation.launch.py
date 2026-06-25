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

For natural-language control ("go to the office"), the felix_llm agent comes up
with this stack (llm:=true by default). It only HTTP-calls the local LLM server,
so it costs nothing until you talk to it -- but you must also be serving the
model (./llm_server.sh) for commands to work. Then: ros2 run felix_llm talk.

Args (in addition to localization.launch.py's port/map/camera/foxglove/...):
  navigation   run the nav2 navigation servers (default true)
  perception   run felix_perception (YOLO + lidar fusion) (default false)
  llm          run the felix_llm natural-language nav agent (default true)
  llm_base_url OpenAI-compatible LLM endpoint (default http://localhost:8080/v1)
  llm_model    model alias served by llama-server (default nemotron-3-nano-4b)
  mcp          run the felix_llm MCP server for browser/web-UI control
               (default true; needs `pip install mcp` on the Jetson)
  mcp_port     MCP server port (default 8000)
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
    perception = LaunchConfiguration("perception")
    foxglove = LaunchConfiguration("foxglove")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    navigation = LaunchConfiguration("navigation")
    llm = LaunchConfiguration("llm")
    llm_base_url = LaunchConfiguration("llm_base_url")
    llm_model = LaunchConfiguration("llm_model")
    mcp = LaunchConfiguration("mcp")
    mcp_port = LaunchConfiguration("mcp_port")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/myserial"),
        DeclareLaunchArgument("map",
                              default_value="/felix-ai-ros/maps/felix_map.yaml"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("perception", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("serial_baudrate", default_value="256000"),
        DeclareLaunchArgument("navigation", default_value="true"),
        DeclareLaunchArgument("llm", default_value="true"),
        DeclareLaunchArgument("llm_base_url",
                              default_value="http://localhost:8080/v1"),
        DeclareLaunchArgument("llm_model", default_value="nemotron-3-nano-4b"),
        DeclareLaunchArgument("mcp", default_value="true"),
        DeclareLaunchArgument("mcp_port", default_value="8000"),

        # Full localization stack (base + EKF + description + lidar + map_server
        # + AMCL + camera + foxglove). Reuses the one-shot localization launch.
        _inc("felix_bringup", ["launch", "localization.launch.py"],
             args={
                 "port": port,
                 "map": map_yaml,
                 "camera": camera,
                 "perception": perception,
                 "foxglove": foxglove,
                 "serial_baudrate": serial_baudrate,
             }),

        # nav2 navigation servers on top.
        _inc("felix_nav", ["launch", "navigation.launch.py"],
             condition=IfCondition(navigation)),

        # Natural-language nav agent (idles until you talk to it; needs
        # ./llm_server.sh serving the model for commands to actually fire).
        _inc("felix_llm", ["launch", "felix_llm.launch.py"],
             condition=IfCondition(llm),
             args={"llm_base_url": llm_base_url, "llm_model": llm_model,
                   "mcp": mcp, "mcp_port": mcp_port}),
    ])
