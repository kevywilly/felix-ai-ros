"""Pure-logic tests for the detection record contract (U5). No ROS required.

Pins the values the detector maps onto vision_msgs/Detection2D: string class_id,
pixel bbox center = midpoint, size = extent.
"""
import numpy as np

from felix_perception import geometry


def test_records_happy():
    recs = geometry.detections_to_records(
        xyxy=[[10, 20, 30, 60]], conf=[0.9], cls=[0], names={0: "person"})
    assert len(recs) == 1
    r = recs[0]
    assert r.class_id == "person" and isinstance(r.class_id, str)
    assert r.score == 0.9
    assert r.cx == 20.0 and r.cy == 40.0
    assert r.size_x == 20.0 and r.size_y == 40.0


def test_records_empty():
    recs = geometry.detections_to_records(np.zeros((0, 4)), [], [])
    assert recs == []


def test_class_id_string_on_names_miss():
    # names lookup fails for index 7 -> stringified int, still a str.
    recs = geometry.detections_to_records(
        [[0, 0, 2, 2]], [0.5], [7], names={0: "person"})
    assert recs[0].class_id == "7" and isinstance(recs[0].class_id, str)


def test_class_id_string_without_names():
    recs = geometry.detections_to_records([[0, 0, 2, 2]], [0.5], [3], names=None)
    assert recs[0].class_id == "3" and isinstance(recs[0].class_id, str)


def test_multiple_detections_preserved():
    recs = geometry.detections_to_records(
        xyxy=[[0, 0, 10, 10], [5, 5, 25, 45]],
        conf=[0.8, 0.6], cls=[0, 56], names={0: "person", 56: "chair"})
    assert [r.class_id for r in recs] == ["person", "chair"]
    assert recs[1].cx == 15.0 and recs[1].size_y == 40.0
