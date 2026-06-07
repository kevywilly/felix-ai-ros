"""Launch the Felix base stack: hardware bridge, ToF, and (optionally) the EKF.

ros2 launch owns SIGINT and shuts everything down cleanly on Ctrl-C (no manual
kill/wait dance). The keyboard teleop is intentionally NOT launched here -- it
reads raw keystrokes from a terminal, which it cannot do under ros2 launch
(launch captures stdio). Run it in the foreground instead:

    ros2 run felix_base teleop

or use start_ros2.sh, which launches this file in the background and runs teleop
in the foreground.

Args:
  port      serial port for the ROSMASTER board (default /dev/myserial)
  use_ekf   run the robot_localization EKF (default true). When true, the bridge
            stops publishing odom->base_link so the EKF owns that transform.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration("port")
    use_ekf = LaunchConfiguration("use_ekf")

    # The bridge publishes the odom->base_link TF only when the EKF is NOT
    # running (otherwise both would fight over that transform).
    bridge_publish_tf = ParameterValue(
        PythonExpression(["'", use_ekf, "' != 'true'"]), value_type=bool)

    ekf_launch = PathJoinSubstitution(
        [FindPackageShare("felix_localization"), "launch", "ekf.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="/dev/myserial",
            description="Serial port for the ROSMASTER board."),
        DeclareLaunchArgument(
            "use_ekf", default_value="true",
            description="Run the robot_localization EKF (fuses /odom + /imu/data)."),

        Node(
            package="felix_base",
            executable="bridge",
            name="rosmaster_bridge_node",
            output="screen",
            parameters=[{"port": port, "publish_tf": bridge_publish_tf}],
        ),
        Node(
            package="felix_base",
            executable="tof",
            name="tof_node",
            output="screen",
        ),

        # Temporary base_link->imu_link transform so the EKF can consume the IMU.
        # Identity is fine here because we only fuse yaw rate (gyro Z), which is
        # invariant to rotation about the shared z-axis. felix_description (URDF)
        # will replace this with the real mounting transform (~90deg yaw, plus the
        # accel-axis fix) once we need accel/heading.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            arguments=["--frame-id", "base_link", "--child-frame-id", "imu_link"],
            condition=IfCondition(use_ekf),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ekf_launch]),
            condition=IfCondition(use_ekf),
        ),
    ])
