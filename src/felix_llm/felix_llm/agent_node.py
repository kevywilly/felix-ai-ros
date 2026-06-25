#!/usr/bin/env python3
"""Natural-language navigation agent.

Listens for a plain-text command on /llm/command (std_msgs/String), asks the
local LLM (Nemotron via llama-server) which tool to call, executes it against
the robot via the shared RobotSkills, and publishes a one-line reply on
/llm/response.

The tools live in felix_llm.skills (shared with the MCP server, so the two
front-ends can't drift). All motion goes through Nav2's NavigateToPose action --
the LLM never writes /cmd_vel. Talk to it with `ros2 run felix_llm talk`, teach
places with `ros2 run felix_llm teach <name>`, or publish to /llm/command from
Foxglove.

Needs localization + Nav2 up (run alongside felix_bringup navigation.launch.py)
and llm_server.sh serving on :8080.
"""
import json

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

from felix_llm.locations import LocationStore
from felix_llm.skills import RobotSkills, openai_tools, default_locations_file
from felix_llm.behaviors import BehaviorRunner
from felix_llm import llm_client


SYSTEM_PROMPT = (
    "You are the navigation interface for a home robot named Felix. "
    "The user speaks to you in plain language and you drive the robot to named "
    "places on its map by calling tools. Rules: "
    "to drive somewhere, call go_to with the place name. "
    "If a place is unknown, call list_places and tell the user which places exist "
    "-- never invent a place or guess coordinates. "
    "To remember the robot's current spot, call save_place. "
    "To report position, call where_am_i. To halt, call stop. "
    "To say what the camera sees now, call what_do_you_see; to recall whether "
    "and where something was seen, call have_you_seen. "
    "For autonomous modes: call find to search for an object, patrol to loop "
    "through saved places, roam to wander; stop ends any of them, and "
    "what_am_i_doing reports the active mode. "
    "Reply in ONE short, friendly sentence. Do not describe the tools."
)


class AgentNode(Node):
    def __init__(self):
        super().__init__('felix_llm_agent')

        self.declare_parameter('locations_file', default_locations_file())
        self.declare_parameter('llm_base_url', 'http://localhost:8080/v1')
        self.declare_parameter('llm_model', 'nemotron-3-nano-4b')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('arrived_radius', 0.75)  # m: "I'm at X" vs "near X"
        self.declare_parameter('max_tool_turns', 4)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.base_url = self.get_parameter('llm_base_url').value
        self.model = self.get_parameter('llm_model').value
        self.max_turns = int(self.get_parameter('max_tool_turns').value)

        store = LocationStore(self.get_parameter('locations_file').value)
        group = ReentrantCallbackGroup()
        self.skills = RobotSkills(
            self, store,
            map_frame=self.get_parameter('map_frame').value,
            base_frame=self.get_parameter('base_frame').value,
            goal_frame=self.get_parameter('goal_frame').value,
            arrived_radius=self.get_parameter('arrived_radius').value,
            callback_group=group)
        self.tools = openai_tools()

        self.create_subscription(
            String, '/llm/command', self._on_command, 10, callback_group=group)
        self.reply_pub = self.create_publisher(String, '/llm/response', 10)

        # Single owner of long-running behaviors (find/patrol/roam). The MCP
        # server does NOT create one -- its tools publish requests that this
        # runner executes, so the two front-ends never fight over motion.
        self.behavior = BehaviorRunner(self, self.skills, self.reply_pub)

        self.get_logger().info(
            f"LLM nav agent ready. model={self.model} url={self.base_url} "
            f"places={store.path}. Send text on /llm/command.")

    # ---- ROS callbacks --------------------------------------------------
    def _on_command(self, msg):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"command: {text!r}")
        try:
            reply = self._run_agent(text)
        except (OSError, ValueError, KeyError) as exc:
            reply = f"Sorry, I couldn't reach my brain ({exc})."
            self.get_logger().error(f"agent error: {exc}")
        self.get_logger().info(f"reply: {reply!r}")
        self.reply_pub.publish(String(data=reply))

    # ---- agent loop -----------------------------------------------------
    def _run_agent(self, user_text):
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_text},
        ]
        for _ in range(self.max_turns):
            msg = llm_client.chat(self.base_url, self.model, messages, self.tools)
            calls = msg.get("tool_calls") or []
            if not calls:
                content = llm_client.strip_think(msg.get("content"))
                return content or "Okay."
            # Echo the assistant turn (with its tool calls) back into context.
            messages.append({
                "role": "assistant",
                "content": msg.get("content") or "",
                "tool_calls": calls,
            })
            for call in calls:
                messages.append({
                    "role": "tool",
                    "tool_call_id": call.get("id", ""),
                    "content": self._dispatch(call),
                })
        return "I got a bit tangled up working that out -- try rephrasing?"

    def _dispatch(self, call):
        fn = call.get("function", {})
        name = fn.get("name", "")
        try:
            args = json.loads(fn.get("arguments") or "{}")
        except json.JSONDecodeError:
            args = {}
        try:
            return json.dumps(self.skills.dispatch(name, args))
        except Exception as exc:  # noqa: BLE001 - report, never crash the loop
            self.get_logger().error(f"tool {name} failed: {exc}")
            return json.dumps({"result": "error", "detail": str(exc)})


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
