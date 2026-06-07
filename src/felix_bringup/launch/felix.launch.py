"""Launch the Felix hardware bridge and ToF array nodes.

This replaces the background-process half of the old start.sh: ros2 launch owns
SIGINT and shuts both nodes down cleanly on Ctrl-C (no manual kill/wait dance).

The keyboard teleop is intentionally NOT launched here -- it reads raw keystrokes
from a terminal, which it cannot do under ros2 launch (launch captures stdio).
Run it in the foreground instead:

    ros2 run felix_base teleop

or use start.sh, which launches this file in the background and runs teleop in
the foreground.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/myserial",
            description="Serial port for the ROSMASTER board.",
        ),
        Node(
            package="felix_base",
            executable="bridge",
            name="rosmaster_bridge_node",
            output="screen",
            parameters=[{"port": port}],
        ),
        Node(
            package="felix_base",
            executable="tof",
            name="tof_node",
            output="screen",
        ),
    ])
