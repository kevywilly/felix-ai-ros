"""Felix nav2 navigation servers (planner + MPPI controller + behaviors + BT).

The autonomy layer. Run ALONGSIDE localization (felix_bringup localization.launch.py
or felix_slam localization.launch.py), which provides map -> odom and /map. This
launch adds the nav2 servers that turn a goal pose into /cmd_vel:

  * controller_server   MPPI (holonomic) + local costmap -> /cmd_vel
  * planner_server      NavFn global planner + global costmap
  * behavior_server     recovery behaviors (spin, backup, drive_on_heading, wait)
  * bt_navigator        behavior-tree orchestration (NavigateToPose / ThroughPoses)
  * lifecycle_manager   configures + activates the above

Send goals from the Foxglove/rviz "Nav2 Goal" tool (the /navigate_to_pose action).

Args:
  params      nav2 params yaml (default: felix_nav/config/nav2_params.yaml)
  autostart   bring the lifecycle nodes up automatically (default true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")
    autostart = LaunchConfiguration("autostart")

    default_params = PathJoinSubstitution(
        [FindPackageShare("felix_nav"), "config", "nav2_params.yaml"])

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params,
                              description="nav2 params yaml."),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Auto-activate the lifecycle nodes."),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[params]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": False,
                 "autostart": autostart,
                 "node_names": lifecycle_nodes,
             }]),
    ])
