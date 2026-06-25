"""Launch the natural-language navigation agent (and optionally the MCP server).

Run this alongside the navigation stack and the LLM server:

    ros2 launch felix_bringup navigation.launch.py     # localization + Nav2
    ./llm_server.sh                                     # Nemotron on :8080
    ros2 launch felix_llm felix_llm.launch.py          # agent only
    ros2 launch felix_llm felix_llm.launch.py mcp:=true   # + MCP server for the web UI

Then: ros2 run felix_llm talk  ->  "go to the kitchen".

Args:
  llm_base_url    OpenAI-compatible endpoint (default http://localhost:8080/v1)
  llm_model       model alias served by llama-server (default nemotron-3-nano-4b)
  locations_file  named-places YAML (default: package share config/locations.yaml)
  mcp             also run the MCP server (browser/web-UI control) (default false)
  mcp_port        MCP server port (default 8000)
  mcp_transport   streamable-http | sse (default streamable-http)
  mcp_cors_origin allowed browser origin (default *; e.g. http://orin1:8080)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "llm_base_url": "http://localhost:8080/v1",
        "llm_model": "nemotron-3-nano-4b",
        "locations_file": "",  # empty -> node default (package share config)
        "mcp": "false",
        "mcp_port": "8000",
        "mcp_transport": "streamable-http",
        "mcp_cors_origin": "*",
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    params = {
        "llm_base_url": LaunchConfiguration("llm_base_url"),
        "llm_model": LaunchConfiguration("llm_model"),
    }

    return LaunchDescription(declared + [
        Node(
            package="felix_llm",
            executable="agent",
            name="felix_llm_agent",
            output="screen",
            parameters=[params],
        ),
        # Optional MCP server (needs `pip install mcp`). It parses CLI args, not
        # ROS params, so its config is passed as `arguments`.
        Node(
            package="felix_llm",
            executable="mcp",
            name="felix_llm_mcp",
            output="screen",
            condition=IfCondition(LaunchConfiguration("mcp")),
            arguments=[
                "--port", LaunchConfiguration("mcp_port"),
                "--transport", LaunchConfiguration("mcp_transport"),
                "--cors-origin", LaunchConfiguration("mcp_cors_origin"),
            ],
        ),
    ])
