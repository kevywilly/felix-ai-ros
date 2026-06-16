"""Pure-logic tests for wedge selection + range clustering (U6). No ROS."""
import numpy as np

from felix_perception import geometry


def test_select_wedge_basic():
    bearings = np.array([-0.5, -0.1, 0.0, 0.1, 0.5])
    idx = geometry.select_wedge(bearings, -0.15, 0.15)
    assert set(idx.tolist()) == {1, 2, 3}


def test_select_wedge_margin():
    bearings = np.array([-0.2, 0.0, 0.2])
    idx = geometry.select_wedge(bearings, -0.1, 0.1, margin=0.15)
    assert set(idx.tolist()) == {0, 1, 2}


def test_cluster_nearest_picks_near_not_mean():
    # near object ~1 m + background ~4 m sharing the wedge: median of the NEAR
    # cluster, never the mean (~2.5) which background corrupts.
    ranges = [1.0, 1.02, 0.98, 4.0, 4.1, 3.95]
    assert abs(geometry.cluster_nearest_range(ranges, gap=0.2) - 1.0) < 1e-6


def test_cluster_empty_returns_none():
    assert geometry.cluster_nearest_range([]) is None
    assert geometry.cluster_nearest_range([float("inf"), -1.0, 0.0]) is None


def test_cluster_min_points_rejects_too_few():
    # A lone near return with min_points=2 is rejected; the 2-point far cluster wins.
    ranges = [1.0, 4.0, 4.05]
    assert abs(geometry.cluster_nearest_range(ranges, gap=0.2, min_points=2)
               - 4.025) < 1e-6


def test_cluster_single_return_ok_with_min_one():
    assert geometry.cluster_nearest_range([2.5]) == 2.5


def test_two_objects_resolve_to_separate_clusters():
    # Two objects at 1.0 and 2.5 m -> nearest cluster median is 1.0.
    ranges = [2.48, 2.5, 2.52, 1.0, 0.99]
    assert abs(geometry.cluster_nearest_range(ranges, gap=0.2) - 0.995) < 1e-6
