"""Run the robot_localization EKF for the Felix base.

Standalone: just the ekf_node + config. felix_bringup includes this and, when it
does, sets the bridge's publish_tf:=false so the two don't both publish
odom -> base_link.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('felix_localization'), 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
