"""Named-location store: a flat name -> (x, y, yaw) map persisted as YAML.

The file is the single source of truth and is re-read on every access, so places
taught by the `teach` CLI, by the agent's save_location tool, or by hand-editing
all show up live without restarting anything. Poses are in the map frame
(REP-103: yaw in radians, CCW positive).
"""
import math
import os

import yaml


class LocationStore:
    def __init__(self, path):
        self.path = os.path.expanduser(path)

    # ---- persistence ----------------------------------------------------
    def _load(self):
        try:
            with open(self.path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}
        locs = data.get('locations') or {}
        return {str(k).strip().lower(): v for k, v in locs.items()}

    def _dump(self, locs):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, 'w') as f:
            yaml.safe_dump({'locations': locs}, f, default_flow_style=False,
                           sort_keys=True)

    # ---- queries --------------------------------------------------------
    def names(self):
        return sorted(self._load().keys())

    def all(self):
        """Full name -> {x, y, yaw} map (lowercased keys)."""
        return self._load()

    def save(self, name, x, y, yaw):
        locs = self._load()
        locs[str(name).strip().lower()] = {
            'x': round(float(x), 4),
            'y': round(float(y), 4),
            'yaw': round(float(yaw), 4),
        }
        self._dump(locs)

    def resolve(self, query):
        """Return (status, payload) for a fuzzy name lookup.

        status is one of:
          'ok'        -> payload = (name, pose_dict)
          'unknown'   -> payload = list of known names
          'ambiguous' -> payload = list of candidate names
        """
        locs = self._load()
        if not locs:
            return 'unknown', []
        q = str(query).strip().lower()
        if q in locs:
            return 'ok', (q, locs[q])
        matches = [n for n in locs if q in n or n in q]
        if len(matches) == 1:
            return 'ok', (matches[0], locs[matches[0]])
        if len(matches) > 1:
            return 'ambiguous', sorted(matches)
        return 'unknown', sorted(locs.keys())

    def nearest(self, x, y):
        """Closest known place to (x, y); returns (name, distance_m) or None."""
        locs = self._load()
        if not locs:
            return None
        name = min(locs, key=lambda n: math.hypot(locs[n]['x'] - x,
                                                   locs[n]['y'] - y))
        d = math.hypot(locs[name]['x'] - x, locs[name]['y'] - y)
        return name, d


def yaw_to_quat(yaw):
    """Planar yaw (rad) -> (z, w) quaternion components (x = y = 0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(z, w):
    """(z, w) quaternion components -> planar yaw (rad)."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
