"""Persistent sighting store: object label -> last-seen record, across restarts.

Perception runs at camera rate, so RobotSkills keeps the working memory in RAM and
flushes here on a timer. Writes are atomic (temp file + rename) and merge with
whatever is already on disk (newest stamp wins per label), so the agent node and
the MCP server -- two processes sharing one file -- converge instead of clobbering
each other.

Stamps are wall-clock epoch seconds, so "seconds_ago" stays meaningful after a
restart. Lives next to locations.yaml (under install/, gitignored, but on the
bind-mounted repo so it survives container restarts).
"""
import os
import tempfile

import yaml

# Only these record fields are persisted (drops any transient extras).
FIELDS = ("stamp", "score", "x", "y", "source", "loc_stamp")


class SightingStore:
    def __init__(self, path):
        self.path = os.path.expanduser(path)

    def mtime(self):
        try:
            return os.path.getmtime(self.path)
        except OSError:
            return None

    def load(self):
        try:
            with open(self.path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}
        except yaml.YAMLError:
            return {}
        raw = data.get("sightings") or {}
        out = {}
        for k, v in raw.items():
            if isinstance(v, dict):
                out[str(k).strip().lower()] = {f: v[f] for f in FIELDS if f in v}
        return out

    def save(self, sightings):
        clean = {k: {f: v[f] for f in FIELDS if f in v}
                 for k, v in sightings.items()}
        d = os.path.dirname(self.path) or "."
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d, prefix=".sightings.", suffix=".yaml")
        try:
            with os.fdopen(fd, "w") as f:
                yaml.safe_dump({"sightings": clean}, f,
                               default_flow_style=False, sort_keys=True)
            os.replace(tmp, self.path)  # atomic
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise


def merge(a, b):
    """Union of two label->record maps; newest stamp wins per label."""
    out = dict(a)
    for k, rec in b.items():
        if k not in out or (rec.get("stamp") or 0) >= (out[k].get("stamp") or 0):
            out[k] = rec
    return out
