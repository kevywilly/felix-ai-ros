#!/usr/bin/env python3
"""MCP server exposing Felix's nav skills to any MCP host (e.g. the llama-server
web UI).

This is the *second* front-end onto the same felix_llm.skills used by the agent
node -- register it in the llama-server web UI's MCP settings (serve llama-server
with --ui-mcp-proxy) and you can drive the robot by chatting in the browser.

    pip install mcp                       # one-time, on the Jetson
    ros2 run felix_llm mcp                # serves streamable-HTTP on :8000/mcp
    ros2 run felix_llm mcp --transport sse --port 8000   # SSE at :8000/sse

It runs an rclpy node (spun in a background thread) so the MCP tool handlers can
call straight into ROS. Needs Nav2 + localization up, same as the agent. All
motion goes through NavigateToPose -- the MCP host never touches /cmd_vel.
"""
import sys
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from felix_llm.locations import LocationStore
from felix_llm.skills import RobotSkills, default_locations_file


def _parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(description="Felix nav MCP server.")
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8000)
    p.add_argument('--transport', choices=['streamable-http', 'sse'],
                   default='streamable-http')
    p.add_argument('--cors-origin', default='*',
                   help="Allowed browser origin(s) (default '*'; e.g. "
                        "http://orin1:8080). The web UI is a different origin "
                        "than this server, so CORS headers are required.")
    p.add_argument('--locations-file', default=default_locations_file())
    p.add_argument('--map-frame', default='map')
    p.add_argument('--base-frame', default='base_link')
    return p.parse_args(remove_ros_args(argv)[1:])


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv)
    args = _parse_args(argv)

    try:
        from mcp.server.fastmcp import FastMCP
        from starlette.middleware.cors import CORSMiddleware
        import uvicorn
    except ImportError:
        sys.exit("The 'mcp' package is required for the MCP server: pip install mcp")

    rclpy.init(args=argv)
    node = rclpy.create_node('felix_llm_mcp')
    store = LocationStore(args.locations_file)
    group = ReentrantCallbackGroup()
    skills = RobotSkills(node, store, map_frame=args.map_frame,
                         base_frame=args.base_frame, callback_group=group)

    # Spin ROS in the background so TF fills and action futures resolve while the
    # MCP event loop owns the main thread. MCP tool handlers (run in worker
    # threads by FastMCP) call into these skills.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    mcp = FastMCP("felix-robot", host=args.host, port=args.port)

    @mcp.tool()
    def go_to(place: str) -> dict:
        """Drive the robot to a saved named place on the map (e.g. 'kitchen')."""
        return skills.go_to(place)

    @mcp.tool()
    def list_places() -> dict:
        """List all saved places with their map coordinates (x, y, yaw)."""
        return skills.list_places()

    @mcp.tool()
    def save_place(name: str) -> dict:
        """Save the robot's current location under a name."""
        return skills.save_place(name)

    @mcp.tool()
    def where_am_i() -> dict:
        """Report the robot's current location relative to known places."""
        return skills.where_am_i()

    @mcp.tool()
    def stop() -> dict:
        """Cancel the current navigation goal and stop the robot."""
        return skills.stop()

    @mcp.tool()
    def what_do_you_see() -> dict:
        """Report what the robot's camera currently sees (live object detections)."""
        return skills.what_do_you_see()

    @mcp.tool()
    def have_you_seen(thing: str) -> dict:
        """Recall whether a kind of object (e.g. 'cat') was seen recently, and where."""
        return skills.have_you_seen(thing)

    @mcp.tool()
    def find(thing: str) -> dict:
        """Search for an object: go to where it was last seen, then other saved places."""
        return skills.find(thing)

    @mcp.tool()
    def patrol() -> dict:
        """Continuously loop through the saved named places until told to stop."""
        return skills.patrol()

    @mcp.tool()
    def roam() -> dict:
        """Wander the mapped area autonomously (Nav2 avoids obstacles) until told to stop."""
        return skills.roam()

    @mcp.tool()
    def what_am_i_doing() -> dict:
        """Report the current autonomous behavior (find / patrol / roam / idle)."""
        return skills.what_am_i_doing()

    # Build the Starlette app for the chosen transport and wrap it in CORS so the
    # browser web UI (a different origin, e.g. :8080) can reach this server
    # directly -- without this the fetch fails with "Failed to fetch". Expose
    # Mcp-Session-Id so the streamable-HTTP client can read the session header.
    app = mcp.sse_app() if args.transport == 'sse' else mcp.streamable_http_app()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[args.cors_origin],
        allow_methods=["*"],
        allow_headers=["*"],
        expose_headers=["Mcp-Session-Id", "mcp-session-id"],
    )

    node.get_logger().info(
        f"Felix MCP server on {args.transport} at {args.host}:{args.port} "
        f"(places={store.path}, cors={args.cors_origin}). Add "
        f"http://<host>:{args.port}/{'sse' if args.transport == 'sse' else 'mcp'} "
        f"in the llama-server web UI MCP settings.")
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
