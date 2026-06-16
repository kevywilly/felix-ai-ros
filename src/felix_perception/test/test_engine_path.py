"""Pure-logic tests for engine cache-path resolution (U4). No ROS required."""
import os

from felix_perception import geometry


def test_explicit_dir_and_bare_name():
    assert geometry.engine_cache_path("yolo11n.pt", "/tmp/eng") \
        == "/tmp/eng/yolo11n.engine"


def test_path_weights_uses_basename_stem():
    assert geometry.engine_cache_path("/models/yolo11n.pt", "/tmp/eng") \
        == "/tmp/eng/yolo11n.engine"


def test_env_override(monkeypatch):
    monkeypatch.setenv("FELIX_ENGINE_DIR", "/data/eng")
    assert geometry.engine_cache_path("yolo11n.pt") \
        == "/data/eng/yolo11n.engine"


def test_default_cache_dir(monkeypatch):
    monkeypatch.delenv("FELIX_ENGINE_DIR", raising=False)
    expected = os.path.join(
        os.path.expanduser(os.path.join("~", ".cache", "felix")),
        "yolo11n.engine")
    assert geometry.engine_cache_path("yolo11n.pt") == expected


def test_explicit_dir_beats_env(monkeypatch):
    monkeypatch.setenv("FELIX_ENGINE_DIR", "/data/eng")
    assert geometry.engine_cache_path("yolo11n.pt", "/explicit") \
        == "/explicit/yolo11n.engine"
