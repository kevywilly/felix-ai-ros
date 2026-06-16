"""Launch felix_perception: the YOLO detector + the lidar fusion node.

Consumes /camera/image_raw/compressed (start the camera via camera.launch.py or
a bringup stack) and /scan; publishes /perception/detections,
/perception/annotated/compressed, and /perception/objects (map-frame markers).

Build the TensorRT engine once first:  ros2 run felix_perception build-engine
Without it the detector falls back to the .pt (slower) -- watch /perception/status.

NOTE: the detector asserts the incoming image size matches the committed
camera_info (640x360 by default, the camera's bringup resolution). If you raise
the camera resolution for perception, re-calibrate at that resolution.

Args:
  weights            YOLO weights name/path (default yolo11n.pt)
  conf               detection confidence threshold (default 0.25)
  imgsz              YOLO input size (default 640)
  publish_annotated  publish the FPV boxes stream (default true)
  require_engine     fail instead of falling back to the .pt (default false)
  engine_dir         engine cache dir ("" = $FELIX_ENGINE_DIR or ~/.cache/felix)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        "weights": "yolo11n.pt",
        "conf": "0.25",
        "imgsz": "640",
        "publish_annotated": "true",
        "require_engine": "false",
        "engine_dir": "",
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    def lc(name):
        return LaunchConfiguration(name)

    detector = Node(
        package="felix_perception",
        executable="detector",
        name="detector_node",
        output="screen",
        parameters=[{
            "weights": lc("weights"),
            "engine_dir": lc("engine_dir"),
            "conf": ParameterValue(lc("conf"), value_type=float),
            "imgsz": ParameterValue(lc("imgsz"), value_type=int),
            "publish_annotated": ParameterValue(
                lc("publish_annotated"), value_type=bool),
            "require_engine": ParameterValue(
                lc("require_engine"), value_type=bool),
        }],
    )
    fusion = Node(
        package="felix_perception",
        executable="fusion",
        name="fusion_node",
        output="screen",
    )
    return LaunchDescription(declared + [detector, fusion])
