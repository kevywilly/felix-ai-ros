"""Long-running autonomous behaviors: find / patrol / roam.

Unlike the one-shot skills (go_to, where_am_i, ...), these run over time --
sequencing NavigateToPose goals, 360 scan-spins, and perception checks until done
or cancelled. The BehaviorRunner is the SINGLE owner of motion-over-time: it lives
only in the agent node, so the agent and the MCP server never fight over driving.
Both front-ends start/stop behaviors by publishing a request on /felix_llm/behavior
(see RobotSkills); the runner is the only subscriber that executes them.

Progress and results are published on /llm/response (so they show up in `talk` /
Foxglove) and on /felix_llm/behavior_status (machine-readable, for the status
tool). All motion goes through Nav2, so obstacle avoidance is the planner's job.

Behaviors run on a plain worker thread (free to block/sleep); the node's
MultiThreadedExecutor resolves the action futures it polls. Each run owns a fresh
cancel Event. The active goal handle is shared with the executor thread (stop()),
so it is guarded by _goal_lock and only ever cleared/cancelled per-handle -- a
stop or a result-timeout always cancels the in-flight Nav2 goal.
"""
import json
import math
import random
import threading
import time

from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String


class BehaviorRunner:
    def __init__(self, node, skills, reply_pub):
        self.node = node
        self.skills = skills
        self.reply_pub = reply_pub          # /llm/response (user-facing)
        self._cancel = threading.Event()
        self._cancel.set()                  # nothing running yet
        self._thread = None
        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._status = "idle"

        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.spin = ActionClient(node, Spin, '/spin')
        self.status_pub = node.create_publisher(String, '/felix_llm/behavior_status', 10)
        node.create_subscription(String, '/felix_llm/behavior', self._on_cmd, 10)

        # Latched occupancy grid for roam (random free-space waypoints).
        self._map = None
        self._free = []
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(OccupancyGrid, '/map', self._on_map, qos)

    # ---- command intake -------------------------------------------------
    def _on_cmd(self, msg):
        try:
            req = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if req.get("action") == "stop":
            self.stop()
        elif req.get("action") == "start":
            self.start(req.get("mode"), req.get("args") or {})

    def start(self, mode, args):
        self.stop()
        ev = threading.Event()
        self._cancel = ev
        self._thread = threading.Thread(
            target=self._run, args=(mode, args, ev), daemon=True)
        self._thread.start()

    def stop(self):
        self._cancel.set()
        with self._goal_lock:
            g, self._active_goal = self._active_goal, None
        if g is not None:
            try:
                g.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self._set_status("idle")

    def _run(self, mode, args, ev):
        try:
            if mode == "find":
                self._find(args, ev)
            elif mode == "patrol":
                self._patrol(args, ev)
            elif mode == "roam":
                self._roam(args, ev)
        except Exception as exc:  # noqa: BLE001 - never crash the node
            self.node.get_logger().error(f"behavior {mode} failed: {exc}")
        finally:
            if not ev.is_set():
                self._set_status("idle")

    # ---- behaviors ------------------------------------------------------
    def _find(self, args, ev):
        thing = str(args.get("thing", "")).strip().lower()
        if not thing:
            self._say("Find what?")
            return
        self._say(f"Looking for the {thing}...")
        # Candidate spots: where it was last seen first, then the saved places.
        spots, seen_xy = [], set()
        _, rec = self.skills.match_sighting(thing)
        if rec and rec.get("x") is not None and rec.get("y") is not None:
            spots.append(("where I last saw it", rec["x"], rec["y"]))
            seen_xy.add((round(rec["x"], 1), round(rec["y"], 1)))
        for name, p in self.skills.store.all().items():
            key = (round(p["x"], 1), round(p["y"], 1))
            if key not in seen_xy:
                spots.append((name, p["x"], p["y"]))
                seen_xy.add(key)
        if not spots:
            self._say("I don't know anywhere to look yet.")
            return
        for label, x, y in spots:
            if ev.is_set():
                return
            self._set_status(f"checking {label} for the {thing}")
            st = self._nav_to(x, y, 0.0, ev)
            if st == "cancelled":
                return
            if st != "arrived":
                continue
            self._scan(ev)
            if self._seen_now(thing, ev):
                pose = self.skills._current_xytheta()
                near = self.skills.store.nearest(pose[0], pose[1]) if pose else None
                where = f" near the {near[0]}" if near else " here"
                self._say(f"Found the {thing}{where}!")
                return
        if not ev.is_set():
            self._say(f"I looked around but couldn't find the {thing}.")

    def _patrol(self, args, ev):
        # Resolve the route ONCE (no per-iteration YAML reload); accept a single
        # place name passed as a bare string as well as a list.
        places_arg = args.get("places")
        if isinstance(places_arg, str):
            places_arg = [places_arg]
        snapshot = self.skills.store.all()
        wanted = list(places_arg) if places_arg else list(snapshot.keys())
        route = [(str(n).strip().lower(), snapshot[str(n).strip().lower()])
                 for n in wanted if str(n).strip().lower() in snapshot]
        if not route:
            self._say("No saved places to patrol.")
            return
        self._say("Starting patrol.")
        while not ev.is_set():
            for name, p in route:
                if ev.is_set():
                    return
                self._set_status(f"patrolling to {name}")
                st = self._nav_to(p["x"], p["y"], p.get("yaw", 0.0), ev)
                if st == "cancelled":
                    return
                if st == "arrived":
                    self._scan(ev)
                elif self._sleep(0.5, ev):   # backoff on reject/fail
                    return

    def _roam(self, args, ev):
        self._say("Roaming -- I'll wander and let Nav2 avoid obstacles.")
        while not ev.is_set():
            pt = self._random_free_point()
            if pt is None:
                self._set_status("roam: waiting for a map")
                if self._sleep(1.0, ev):
                    return
                continue
            self._set_status(f"roaming to ({pt[0]:.1f}, {pt[1]:.1f})")
            st = self._nav_to(pt[0], pt[1], random.uniform(-math.pi, math.pi), ev)
            if st == "cancelled":
                return
            if st != "arrived" and self._sleep(0.5, ev):  # backoff on reject/fail
                return

    # ---- perception helper ---------------------------------------------
    def _seen_now(self, thing, ev, window=3.0):
        end = time.time() + window
        while time.time() < end:
            if ev.is_set():
                return False
            if self.skills.seeing_now(thing):
                return True
            time.sleep(0.2)
        return False

    # ---- nav / spin plumbing -------------------------------------------
    def _nav_to(self, x, y, yaw, ev):
        if not self.nav.wait_for_server(timeout_sec=3.0):
            self._say("Navigation isn't running (is Nav2 up?).")
            return "no_nav"
        goal = NavigateToPose.Goal()
        goal.pose = self.skills._pose_stamped(x, y, yaw)
        handle = self._await(self.nav.send_goal_async(goal), ev, 10.0)
        if handle is None:
            return "cancelled" if ev.is_set() else "failed"
        if not handle.accepted:
            return "rejected"
        self._set_goal(handle, ev)
        res = self._await(handle.get_result_async(), ev, 180.0)
        self._clear_goal(handle, cancel=(res is None))  # cancel on timeout/abort
        if res is None:
            return "cancelled" if ev.is_set() else "failed"
        return "arrived" if res.status == GoalStatus.STATUS_SUCCEEDED else "failed"

    def _scan(self, ev, yaw=2 * math.pi):
        """360 spin to sweep the narrow camera FOV; skipped if /spin isn't up."""
        if ev.is_set() or not self.spin.wait_for_server(timeout_sec=2.0):
            return
        goal = Spin.Goal()
        goal.target_yaw = float(yaw)
        handle = self._await(self.spin.send_goal_async(goal), ev, 5.0)
        if handle is None or not handle.accepted:
            return
        self._set_goal(handle, ev)
        res = self._await(handle.get_result_async(), ev, 30.0)
        self._clear_goal(handle, cancel=(res is None))

    def _set_goal(self, handle, ev):
        """Register the in-flight goal so stop() can cancel it. If a stop already
        landed for this run, cancel immediately instead of storing it."""
        with self._goal_lock:
            if ev.is_set():
                cancel = True
            else:
                self._active_goal = handle
                cancel = False
        if cancel:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def _clear_goal(self, handle, cancel):
        with self._goal_lock:
            same = self._active_goal is handle
            if same:
                self._active_goal = None
        if cancel:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def _await(self, future, ev, timeout):
        """Poll an action future until done/cancel/timeout (worker thread)."""
        end = time.time() + timeout
        while not future.done():
            if ev.is_set() or time.time() > end:
                return None
            time.sleep(0.1)
        return future.result()

    def _sleep(self, dur, ev):
        """Interruptible sleep; returns True if cancelled."""
        end = time.time() + dur
        while time.time() < end:
            if ev.is_set():
                return True
            time.sleep(0.1)
        return False

    # ---- map / roam helpers --------------------------------------------
    def _on_map(self, msg):
        self._map = msg
        self._free = [i for i, v in enumerate(msg.data) if v == 0]

    def _random_free_point(self):
        free, grid = self._free, self._map   # local refs (rebound, not mutated)
        if grid is None or not free:
            return None
        info = grid.info
        idx = random.choice(free)
        cy, cx = divmod(idx, info.width)
        x = info.origin.position.x + (cx + 0.5) * info.resolution
        y = info.origin.position.y + (cy + 0.5) * info.resolution
        return (x, y)

    # ---- status / messaging --------------------------------------------
    def _say(self, text):
        self.reply_pub.publish(String(data=text))
        self.node.get_logger().info(f"behavior: {text}")

    def _set_status(self, text):
        self._status = text
        self.status_pub.publish(String(data=text))
