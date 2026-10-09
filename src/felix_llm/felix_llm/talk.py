#!/usr/bin/env python3
"""Talk to the LLM nav agent from the terminal.

    ros2 run felix_llm talk                 # interactive prompt
    ros2 run felix_llm talk go to the kitchen   # one-shot

Publishes your text on /llm/command and prints the agent's /llm/response. The
agent node (ros2 run felix_llm agent) must be running.
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talk(Node):
    def __init__(self):
        super().__init__('felix_llm_talk')
        self.pub = self.create_publisher(String, '/llm/command', 10)
        self.create_subscription(String, '/llm/response', self._on_reply, 10)
        self._got_reply = False

    def _on_reply(self, msg):
        self._got_reply = True
        print(f"\nFelix: {msg.data}\n")

    def send(self, text):
        self._got_reply = False
        self.pub.publish(String(data=text))

    def wait_reply(self, timeout=90.0):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and not self._got_reply:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > end:
                print("\n(no reply -- is the agent node running and llama-server up?)\n")
                return


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    rclpy.init()
    node = Talk()
    # Give pub/sub a moment to discover the agent.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    try:
        if argv:
            node.send(" ".join(argv))
            node.wait_reply()
        else:
            print("Talk to Felix (Ctrl-D / Ctrl-C to quit). e.g. 'go to the kitchen'")
            while rclpy.ok():
                try:
                    text = input("you> ").strip()
                except EOFError:
                    break
                if not text:
                    continue
                node.send(text)
                node.wait_reply()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
