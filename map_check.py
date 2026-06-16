#!/usr/bin/env python3
"""map_check -- turn "the map looks wonky" into numbers.

The complaint "looks good live, bad in Nav2" has at least three distinct,
*measurable* causes, each with its own signature:

  * trinary erosion  -> the saved grid has far MORE unknown (grey) cells than the
                        live map; thin walls drop out. (Fixed by saving --mode scale.)
  * stale snapshot    -> saved occupied-cell count is lower / geometry lags live.
  * resume double wall-> occupied-cell count is HIGHER and walls are thick/doubled.

This tool reports the occupied/free/unknown cell breakdown (and a crude wall-
thickness proxy) for a SAVED map, and -- with --live -- for slam_toolbox's live
/map, then diffs them. Use it before/after a config change so you can tell which
fix actually helped instead of eyeballing Foxglove.

Usage:
  ./map_check.py [map.yaml|map.png|map.pgm]          # stats for a saved map
  ./map_check.py --live                              # one-shot live /map stats
  ./map_check.py maps/felix_map.yaml --live          # saved vs live, with a diff

Saved default: /felix-ai-ros/maps/felix_map.yaml
Live needs a sourced ROS env with slam_toolbox (or AMCL/map_server) publishing /map.
"""
import argparse
import os
import sys

import numpy as np


def _classify(occupied, free, unknown, total):
    """Pretty one-block report from cell counts."""
    pct = lambda n: 100.0 * n / total if total else 0.0
    print(f"  cells       : {total:,} ({pct(occupied):.1f}% occ / "
          f"{pct(free):.1f}% free / {pct(unknown):.1f}% unknown)")
    print(f"  occupied    : {occupied:,}")
    print(f"  free        : {free:,}")
    print(f"  unknown     : {unknown:,}")
    return dict(occupied=occupied, free=free, unknown=unknown, total=total,
                occ_pct=pct(occupied), unk_pct=pct(unknown))


def _wall_thickness(occ_mask):
    """Crude doubled-wall proxy: mean occupied run-length along rows.

    Single clean walls are ~1-2 cells thick; resume double-walls and smear push
    this up. Comparative only -- watch the trend across saves, not the absolute.
    """
    runs = []
    for row in occ_mask:
        if not row.any():
            continue
        # lengths of contiguous True runs
        idx = np.flatnonzero(np.diff(np.concatenate(([0], row.view(np.int8), [0]))))
        runs.extend((idx[1::2] - idx[0::2]).tolist())
    return float(np.mean(runs)) if runs else 0.0


def stats_from_saved(path):
    """Cell breakdown for a saved map image (.png/.pgm), resolving .yaml -> image."""
    try:
        import cv2
    except ImportError:
        sys.exit("map_check: needs opencv (cv2) to read the saved image.")

    # Resolve a .yaml to its referenced image; thresholds come from the yaml too.
    occ_t, free_t = 0.65, 0.196
    if path.endswith(".yaml"):
        import yaml
        with open(path) as fh:
            meta = yaml.safe_load(fh)
        occ_t = float(meta.get("occupied_thresh", occ_t))
        free_t = float(meta.get("free_thresh", free_t))
        img_rel = meta["image"]
        path = img_rel if os.path.isabs(img_rel) else os.path.join(
            os.path.dirname(os.path.abspath(path)), img_rel)

    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"map_check: cannot read image {path}")

    # Split off an alpha channel if present (scale-mode PNG encodes unknown as
    # alpha==0); otherwise trinary uses the grey 205 sentinel for unknown.
    alpha = None
    if img.ndim == 3 and img.shape[2] == 4:
        alpha = img[:, :, 3]
        gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_BGR2GRAY)
    elif img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    p = gray.astype(np.float32) / 255.0          # map_saver writes 255*(1-occ)
    occ = 1.0 - p                                  # back to occupancy probability
    if alpha is not None:
        unknown_mask = alpha == 0
    else:
        unknown_mask = np.isclose(gray, 205, atol=3)   # trinary grey sentinel
    occ_mask = (occ >= occ_t) & ~unknown_mask
    free_mask = (occ <= free_t) & ~unknown_mask

    print(f"SAVED  {path}")
    s = _classify(int(occ_mask.sum()), int(free_mask.sum()),
                  int(unknown_mask.sum()), gray.size)
    s["wall"] = _wall_thickness(occ_mask)
    print(f"  wall thick  : {s['wall']:.2f} cells (mean occupied run; doubled walls raise it)")
    return s


def stats_from_live(timeout=10.0):
    """One-shot snapshot of the live /map OccupancyGrid -> same cell breakdown."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from nav_msgs.msg import OccupancyGrid
    except ImportError:
        sys.exit("map_check: --live needs a sourced ROS 2 env (rclpy, nav_msgs).")

    import time
    rclpy.init()
    node = Node("map_check")
    got = {}
    # /map is latched transient-local; match it or we never receive the last msg.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map",
                             lambda m: got.setdefault("msg", m), qos)
    start = time.monotonic()
    while "msg" not in got and time.monotonic() - start < timeout and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if "msg" not in got:
        sys.exit("map_check: no /map received -- is mapping/localization running?")

    m = got["msg"]
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    unknown_mask = g < 0
    occ_mask = g >= 65            # 0-100 scale; 65 ~= occupied_thresh 0.65
    free_mask = (g >= 0) & (g <= 19)   # ~free_thresh 0.196
    print(f"LIVE   /map  ({m.info.width}x{m.info.height} @ {m.info.resolution} m/cell)")
    s = _classify(int(occ_mask.sum()), int(free_mask.sum()),
                  int(unknown_mask.sum()), g.size)
    s["wall"] = _wall_thickness(occ_mask)
    print(f"  wall thick  : {s['wall']:.2f} cells")
    return s


def diff(live, saved):
    print("\nDIFF  (saved - live)")
    d_unk = saved["unk_pct"] - live["unk_pct"]
    d_occ = saved["occupied"] - live["occupied"]
    print(f"  unknown%    : {d_unk:+.1f} pts", end="")
    print("   <- large positive = trinary erosion (use --mode scale)" if d_unk > 5 else "")
    print(f"  occupied    : {d_occ:+,} cells", end="")
    if d_occ < -0.05 * max(live["occupied"], 1):
        print("   <- saved has fewer walls: stale snapshot or threshold loss")
    elif d_occ > 0.10 * max(live["occupied"], 1):
        print("   <- saved has MORE walls: doubled/smeared (resume or drift)")
    else:
        print("")
    print(f"  wall thick  : {saved['wall'] - live['wall']:+.2f} cells "
          "(positive = thicker/doubled in the saved map)")


def main():
    ap = argparse.ArgumentParser(description="Measure map quality: occupied/free/unknown + wall thickness.")
    ap.add_argument("map", nargs="?", default="/felix-ai-ros/maps/felix_map.yaml",
                    help="saved map (.yaml/.png/.pgm); default maps/felix_map.yaml")
    ap.add_argument("--live", action="store_true", help="also snapshot the live /map and diff")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for /map")
    args = ap.parse_args()

    saved = stats_from_saved(args.map) if os.path.exists(args.map) else None
    if saved is None and not args.live:
        sys.exit(f"map_check: {args.map} not found (and --live not given).")
    live = stats_from_live(args.timeout) if args.live else None
    if live and saved:
        diff(live, saved)


if __name__ == "__main__":
    main()
