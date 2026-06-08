"""Launch the CSI IMX219 camera node -> camera/image_raw/compressed (JPEG).

View in Foxglove: add an Image panel on /camera/image_raw/compressed.

Defaults are tuned for LOW LAG over WiFi (FPV driving), not image quality:
640x360 @ 20 fps, JPEG q60 -- a small fraction of the raw 960x540@59 data.
Raise output_width/output_height/max_fps/jpeg_quality for a sharper image when
on a wired/fast link or for perception.

Args:
  output_width/output_height  published frame size (default 640x360)
  max_fps                     publish rate cap (default 20; 0 = full sensor rate)
  jpeg_quality                JPEG quality 0-100 (default 60)
  flip_method                 nvvidconv flip (default 2 = 180deg, camera upside down)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "output_width": "640",
        "output_height": "360",
        "max_fps": "20.0",
        "jpeg_quality": "60",
        "flip_method": "2",
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    params = {k: LaunchConfiguration(k) for k in args}

    return LaunchDescription(declared + [
        Node(
            package="felix_camera",
            executable="camera",
            name="camera_node",
            output="screen",
            parameters=[params],
        ),
    ])
