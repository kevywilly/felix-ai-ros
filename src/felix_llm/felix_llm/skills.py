"""Robot skills shared by every transport (the LLM agent AND the MCP server).

One implementation, two front-ends -- the felix_llm agent (/llm/command) and the
MCP server (llama-server web UI) both call these methods, so behavior can't drift
between them. Each method returns a plain dict (JSON-serializable): the agent
json.dumps it into a tool result; the MCP server returns it straight to the host.

Navigation goes through Nav2's NavigateToPose action -- skills never write
/cmd_vel. Pose comes from TF (map -> base_link), published continuously, rather
than the latched/event-driven /amcl_pose.

Perception ("what do you see", "have you seen the cat?") is a passive memory fed
by felix_perception. The memory dict (_seen) is read/written from several executor
threads AND the behavior worker thread, so all access goes through _seen_lock.

Long-running behaviors (find/patrol/roam) are owned by a single BehaviorRunner in
the agent node; these tools just publish a request on /felix_llm/behavior.
"""
import json
import os
import threading

import rclpy.time
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray

from felix_llm.locations import yaw_to_quat, quat_to_yaw
from felix_llm.sightings import SightingStore, merge as merge_sightings


# OpenAI / MCP-agnostic tool descriptions. The agent feeds these to the chat API;
# the MCP server reuses the same names/params via the @mcp.tool decorators.
TOOL_SPECS = [
    ("go_to", "Drive the robot to a saved named place on the map.",
     {"place": "name of the place, e.g. 'kitchen'"}),
    ("list_places",
     "List all saved places with their map coordinates (x, y, yaw).", {}),
    ("save_place", "Save the robot's current location under a name.",
     {"name": "name to save the current spot as"}),
    ("where_am_i", "Report the robot's current location relative to known places.", {}),
    ("stop", "Cancel the current navigation goal and stop the robot.", {}),
    ("what_do_you_see",
     "Report what the robot's camera currently sees (live object detections).", {}),
    ("have_you_seen",
     "Recall whether a kind of object was seen recently, and where.",
     {"thing": "object to recall, e.g. 'cat'"}),
    ("find",
     "Search for an object: drive to where it was last seen and look, then try "
     "other saved places until found. Reports back when done.",
     {"thing": "object to find, e.g. 'cat'"}),
    ("patrol",
     "Continuously drive a loop through the saved named places until told to stop.",
     {}),
    ("roam",
     "Wander the mapped area autonomously, letting Nav2 avoid obstacles, until "
     "told to stop.", {}),
    ("what_am_i_doing",
     "Report the current autonomous behavior (find / patrol / roam / idle).", {}),
]


def _label_match(query, label):
    """Word-aware object-label match -- avoids 'cat' matching 'cattle'.

    True on an exact match or when the two labels share a whole word, so
    'the cat' matches 'cat' but 'cat' does not match 'cattle'/'scatter'.
    """
    q = str(query).strip().lower()
    lab = str(label).strip().lower()
    if q == lab:
        return True
    return bool(set(q.split()) & set(lab.split()))


class RobotSkills:
    def __init__(self, node, store, *, map_frame='map', base_frame='base_link',
                 goal_frame='map', arrived_radius=0.75, detection_fresh=3.0,
                 sightings_file=None, sightings_flush=5.0, callback_group=None):
        self.node = node
        self.store = store
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.goal_frame = goal_frame
        self.arrived_radius = arrived_radius
        self.detection_fresh = detection_fresh  # s: "currently seeing" window
        self._goal_handle = None
        self._cancel_requested = False  # closes the go_to send/accept stop window

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose',
                                callback_group=callback_group)

        # Perception memory, persisted across restarts. _seen is touched from
        # multiple executor threads (ReentrantCallbackGroup) and the behavior
        # worker thread, so every access is guarded by _seen_lock.
        sf = sightings_file or os.path.join(
            os.path.dirname(store.path) or '.', 'sightings.yaml')
        self._sightings = SightingStore(sf)
        self._seen_lock = threading.Lock()
        self._seen = self._sightings.load()  # label -> {stamp, score, x, y, ...}
        self._sightings_mtime = self._sightings.mtime()
        self._dirty = False
        self._current = {}         # label -> score, from the latest frame only
        self._last_det_stamp = None
        node.create_subscription(Detection2DArray, '/perception/detections',
                                 self._on_detections, 10,
                                 callback_group=callback_group)
        node.create_subscription(MarkerArray, '/perception/objects',
                                 self._on_objects, 10,
                                 callback_group=callback_group)
        node.create_timer(sightings_flush, self._sync_sightings,
                          callback_group=callback_group)

        # Long-running behaviors are executed by the single BehaviorRunner in the
        # agent node. These tools publish a request; status returns on
        # /felix_llm/behavior_status.
        self.behavior_pub = node.create_publisher(String, '/felix_llm/behavior', 10)
        self._behavior_status = "idle"
        node.create_subscription(String, '/felix_llm/behavior_status',
                                 self._on_behavior_status, 10,
                                 callback_group=callback_group)

    # ---- navigation tools ----------------------------------------------
    def list_places(self):
        locs = self.store.all()
        places = [
            {"name": n, "x": round(p["x"], 2), "y": round(p["y"], 2),
             "yaw": round(p.get("yaw", 0.0), 2)}
            for n, p in sorted(locs.items())
        ]
        return {"places": places}

    def go_to(self, place):
        if not place:
            return {"result": "error", "detail": "no place given"}
        status, payload = self.store.resolve(place)
        if status == "unknown":
            return {"result": "unknown_place", "query": place,
                    "known_places": payload or []}
        if status == "ambiguous":
            return {"result": "ambiguous", "candidates": payload}
        name, pose = payload
        self._publish_behavior("stop")   # a manual destination preempts a mode
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return {"result": "nav_unavailable",
                    "detail": "NavigateToPose server not up (is Nav2 running?)"}
        self._cancel_requested = False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(pose['x'], pose['y'], pose.get('yaw', 0.0))
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        return {"result": "navigating", "place": name,
                "x": pose['x'], "y": pose['y']}

    def save_place(self, name):
        if not name:
            return {"result": "error", "detail": "no name given"}
        pose = self._current_xytheta()
        if pose is None:
            return {"result": "no_pose", "detail": self._no_pose_detail()}
        x, y, yaw = pose
        self.store.save(name, x, y, yaw)
        return {"result": "saved", "name": name.strip().lower(),
                "x": round(x, 2), "y": round(y, 2)}

    def where_am_i(self):
        pose = self._current_xytheta()
        if pose is None:
            return {"result": "no_pose", "detail": self._no_pose_detail()}
        x, y, _ = pose
        out = {"result": "pose", "x": round(x, 2), "y": round(y, 2)}
        near = self.store.nearest(x, y)
        if near is not None:
            name, dist = near
            out["nearest_place"] = name
            out["distance_m"] = round(dist, 2)
            out["at_place"] = dist <= self.arrived_radius
        return out

    def stop(self):
        self._publish_behavior("stop")   # halt any find/patrol/roam
        self._cancel_requested = True    # cancel a go_to goal even if not yet accepted
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        return {"result": "stopping"}

    # ---- perception tools ----------------------------------------------
    def what_do_you_see(self):
        if self._last_det_stamp is None:
            return {"result": "no_perception",
                    "detail": "perception not running (launch with perception:=true)"}
        if (self._now() - self._last_det_stamp) > self.detection_fresh:
            return {"result": "ok", "seeing": [],
                    "detail": "nothing in view right now"}
        seeing = [{"thing": k, "confidence": round(v, 2)}
                  for k, v in sorted(self._current.items(), key=lambda kv: -kv[1])]
        return {"result": "ok", "seeing": seeing}

    def have_you_seen(self, thing):
        if not thing:
            return {"result": "error", "detail": "no object given"}
        match, rec = self.match_sighting(thing)
        if match is None:
            # Persisted memory answers even if perception is currently down; only
            # say "no perception" when we have never seen anything at all.
            if self._last_det_stamp is None and not self.has_any_sighting():
                return {"result": "no_perception",
                        "detail": "perception not running (launch with perception:=true)"}
            return {"seen": False, "thing": str(thing).strip().lower(),
                    "recently_seen": self.seen_labels()}
        out = {"seen": True, "thing": match,
               "seconds_ago": max(0, int(self._now() - rec.get("stamp", self._now()))),
               "confidence": rec.get("score")}
        x, y = rec.get("x"), rec.get("y")
        if x is not None and y is not None:  # need a full coordinate to place it
            out["x"], out["y"] = x, y
            near = self.store.nearest(x, y)
            if near is not None:
                out["near"] = near[0]
        return out

    # ---- shared sighting queries (lock-guarded; return copies) ----------
    def match_sighting(self, thing):
        """(label, record-copy) for the best match, or (None, None)."""
        q = str(thing).strip().lower()
        with self._seen_lock:
            if q in self._seen:
                return q, dict(self._seen[q])
            cands = [k for k in self._seen if _label_match(q, k)]
            if cands:
                best = max(cands, key=lambda k: self._seen[k].get("stamp", 0))
                return best, dict(self._seen[best])
        return None, None

    def seen_labels(self):
        with self._seen_lock:
            return sorted(self._seen.keys())

    def has_any_sighting(self):
        with self._seen_lock:
            return bool(self._seen)

    def seeing_now(self, thing):
        """Is `thing` in the latest detection frame? (_current is rebound, not
        mutated, so a local reference is safe without the lock.)"""
        q = str(thing).strip().lower()
        return any(_label_match(q, k) for k in list(self._current.keys()))

    # ---- perception plumbing -------------------------------------------
    def _on_detections(self, msg):
        now = self._now()
        self._last_det_stamp = now
        # Non-blocking TF: this fires at camera rate, so never wait on the lookup.
        robot = self._current_xytheta(timeout=0.0)
        cur = {}
        for det in msg.detections:
            if not det.results:
                continue
            label = str(det.results[0].hypothesis.class_id).strip().lower()
            if not label:
                continue
            score = float(det.results[0].hypothesis.score)
            if label not in cur or score > cur[label]:
                cur[label] = score
        if cur:
            with self._seen_lock:
                for label, score in cur.items():
                    rec = self._seen.setdefault(label, {})
                    rec["stamp"] = now
                    rec["score"] = round(score, 2)
                    # Robot-pose fallback only when there is no object-fusion fix;
                    # never downgrade a precise 'object' location to robot pose.
                    if robot is not None and rec.get("source") != "object":
                        rec["x"], rec["y"] = round(robot[0], 2), round(robot[1], 2)
                        rec["source"] = "robot"
                        rec["loc_stamp"] = now
            self._dirty = True
        self._current = cur

    def _on_objects(self, msg):
        now = self._now()
        updated = False
        with self._seen_lock:
            for m in msg.markers:
                # The text marker (ns perception_labels) carries label + pose.
                if m.ns != "perception_labels" or not m.text:
                    continue
                rec = self._seen.setdefault(m.text.strip().lower(), {})
                rec["x"] = round(m.pose.position.x, 2)
                rec["y"] = round(m.pose.position.y, 2)
                rec["source"] = "object"
                rec["loc_stamp"] = now
                rec["stamp"] = now
                updated = True
        if updated:
            self._dirty = True

    def _sync_sightings(self):
        """Merge memory with the on-disk store and flush if we have new sightings.

        Reloading lets the agent + MCP processes pick up each other's sightings;
        merge-on-write (newest stamp wins) keeps them from clobbering. A cheap
        mtime check skips the disk read when nothing changed.
        """
        mtime = self._sightings.mtime()
        if not self._dirty and mtime == self._sightings_mtime:
            return
        try:
            disk = self._sightings.load()
        except OSError:
            disk = {}
        snapshot = None
        with self._seen_lock:
            self._seen = merge_sightings(disk, self._seen)
            if self._dirty:
                self._dirty = False
                snapshot = {k: dict(v) for k, v in self._seen.items()}
        if snapshot is not None:
            try:
                self._sightings.save(snapshot)
            except OSError as exc:
                self._dirty = True
                self.node.get_logger().warning(f"sighting save failed: {exc}")
        self._sightings_mtime = self._sightings.mtime()

    def _now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    # ---- long-running behaviors (routed to the single BehaviorRunner) ---
    def find(self, thing):
        if not thing:
            return {"result": "error", "detail": "no object given"}
        if not self._runner_available():
            return self._no_runner()
        t = str(thing).strip().lower()
        self._publish_behavior("start", "find", {"thing": t})
        return {"result": "searching", "thing": t,
                "detail": "I'll look for it and report back."}

    def patrol(self, places=None):
        if not self._runner_available():
            return self._no_runner()
        args = {"places": places} if places else {}
        self._publish_behavior("start", "patrol", args)
        return {"result": "patrolling",
                "detail": "Cycling through the saved places until told to stop."}

    def roam(self):
        if not self._runner_available():
            return self._no_runner()
        self._publish_behavior("start", "roam", {})
        return {"result": "roaming",
                "detail": "Wandering the map and avoiding obstacles until told to stop."}

    def what_am_i_doing(self):
        return {"behavior": self._behavior_status}

    def _runner_available(self):
        # The BehaviorRunner subscribes to /felix_llm/behavior; if nothing is
        # subscribed, the agent (and thus the runner) isn't up.
        return self.behavior_pub.get_subscription_count() > 0

    def _no_runner(self):
        return {"result": "unavailable",
                "detail": "the behavior runner isn't running -- start the "
                          "navigation stack / agent node."}

    def _publish_behavior(self, action, mode=None, args=None):
        self.behavior_pub.publish(String(data=json.dumps(
            {"action": action, "mode": mode, "args": args or {}})))

    def _on_behavior_status(self, msg):
        self._behavior_status = msg.data

    # ---- pose / nav plumbing -------------------------------------------
    def _current_xytheta(self, timeout=0.5):
        """(x, y, yaw) of base_link in the map frame from TF, or None.

        timeout=0.0 for the camera-rate detection path (never block); the default
        0.5 s lets user-initiated tools wait briefly for a fresh transform.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=timeout))
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, quat_to_yaw(q.z, q.w)

    def _no_pose_detail(self):
        return (f"no {self.map_frame}->{self.base_frame} TF "
                f"(is localization up and seeded?)")

    def _pose_stamped(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.goal_frame
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        z, w = yaw_to_quat(float(yaw))
        ps.pose.orientation.z = z
        ps.pose.orientation.w = w
        return ps

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().warn("navigation goal rejected")
            return
        if self._cancel_requested:   # a stop landed between send and accept
            handle.cancel_goal_async()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_handle = None
        self.node.get_logger().info("navigation goal finished")

    # ---- single dispatch point so every front-end behaves identically ---
    def dispatch(self, name, args=None):
        args = args or {}
        if name == "go_to":
            return self.go_to(args.get("place", ""))
        if name == "list_places":
            return self.list_places()
        if name == "save_place":
            return self.save_place(args.get("name", ""))
        if name == "where_am_i":
            return self.where_am_i()
        if name == "stop":
            return self.stop()
        if name == "what_do_you_see":
            return self.what_do_you_see()
        if name == "have_you_seen":
            return self.have_you_seen(args.get("thing", ""))
        if name == "find":
            return self.find(args.get("thing", ""))
        if name == "patrol":
            return self.patrol(args.get("places"))
        if name == "roam":
            return self.roam()
        if name == "what_am_i_doing":
            return self.what_am_i_doing()
        return {"result": "error", "detail": f"unknown tool {name}"}


def openai_tools():
    """The TOOL_SPECS rendered as OpenAI-style function tool schemas."""
    tools = []
    for name, desc, params in TOOL_SPECS:
        props = {k: {"type": "string", "description": v} for k, v in params.items()}
        tools.append({"type": "function", "function": {
            "name": name, "description": desc,
            "parameters": {"type": "object", "properties": props,
                           "required": list(params.keys())}}})
    return tools


def default_locations_file():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('felix_llm'), 'config', 'locations.yaml')
