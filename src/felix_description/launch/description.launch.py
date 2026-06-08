"""Publish the Felix robot model: process the xacro and run robot_state_publisher.

robot_state_publisher reads the URDF and publishes the fixed-joint transforms
(base_footprint->base_link->{imu_link, laser}) on /tf_static. This is the single
source of the robot's TF tree -- the EKF consumes base_link->imu_link and SLAM
consumes base_link->laser from here, replacing bringup's temporary static TFs.

Args:
  use_sim_time   use the /clock topic (default false; set true under a bag/sim)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("felix_description"), "urdf", "felix.urdf.xacro"])
    # `xacro <file>` -> URDF string. value_type=str keeps it a string parameter.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock simulation time."),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
