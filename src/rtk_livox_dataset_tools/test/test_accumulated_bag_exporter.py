"""Tests for causal accumulated dataset export helpers."""

from types import SimpleNamespace
import struct

import numpy as np

from rtk_livox_dataset_tools.accumulated_bag_exporter import (
    CausalCloudAccumulator,
    _append_time_ordered,
    _latest_at_or_before,
    pointcloud2_to_xyzi,
    reduce_points,
)


def _field(name, offset, datatype=7, count=1):
    return SimpleNamespace(
        name=name,
        offset=offset,
        datatype=datatype,
        count=count,
    )


def test_pointcloud_decoder_handles_livox_layout_and_row_padding():
    point_step = 18
    row_step = 2 * point_step + 4
    data = bytearray(2 * row_step)
    values = [
        (1.0, 2.0, 3.0, 4.0, 5, 6),
        (7.0, 8.0, 9.0, 10.0, 11, 12),
        (13.0, 14.0, 15.0, 16.0, 17, 18),
        (float("nan"), 20.0, 21.0, 22.0, 23, 24),
    ]
    offsets = [0, point_step, row_step, row_step + point_step]
    for offset, value in zip(offsets, values):
        struct.pack_into("<ffffBB", data, offset, *value)
    message = SimpleNamespace(
        is_bigendian=False,
        fields=[
            _field("x", 0),
            _field("y", 4),
            _field("z", 8),
            _field("intensity", 12),
            _field("tag", 16, datatype=2),
            _field("line", 17, datatype=2),
        ],
        point_step=point_step,
        row_step=row_step,
        height=2,
        width=2,
        data=data,
    )

    points = pointcloud2_to_xyzi(message)

    np.testing.assert_allclose(
        points,
        [[1, 2, 3, 4], [7, 8, 9, 10], [13, 14, 15, 16]],
    )
    assert points.dtype == np.float32


def test_accumulator_is_causal_end_aligned_and_overlapping():
    accumulator = CausalCloudAccumulator(
        accumulation_sec=0.1,
        output_rate_hz=10.0,
    )
    packet_0 = np.full((1, 4), 0.0, dtype=np.float32)
    packet_1 = np.full((1, 4), 1.0, dtype=np.float32)
    packet_2 = np.full((1, 4), 2.0, dtype=np.float32)
    packet_3 = np.full((1, 4), 3.0, dtype=np.float32)

    accumulator.add_cloud(0.0, packet_0)
    accumulator.add_cloud(0.05, packet_1)
    assert list(accumulator.pop_due(0.1)) == []
    accumulator.add_cloud(0.1, packet_2)
    first = list(accumulator.pop_due(0.11))[0]
    assert np.isclose(first[0], 0.1)
    assert len(first[2]) == 2
    assert first[2][0] is packet_1
    assert first[2][1] is packet_2

    accumulator.add_cloud(0.15, packet_3)
    second = list(accumulator.pop_through(0.2))[0]
    assert np.isclose(second[0], 0.2)
    assert len(second[2]) == 1
    assert second[2][0] is packet_3


def test_point_reduction_is_deterministic():
    points = np.array(
        [
            [0.01, 0.01, 0.01, 1.0],
            [0.02, 0.02, 0.02, 2.0],
            [1.01, 0.01, 0.01, 3.0],
            [2.01, 0.01, 0.01, 4.0],
        ],
        dtype=np.float32,
    )
    reduced = reduce_points(points, voxel_size=0.1, max_points=2)
    np.testing.assert_allclose(reduced[:, 3], [1.0, 4.0])


def test_latest_rtk_selection_never_uses_future_sample():
    stamps = [1.0, 1.2, 1.4]
    samples = ["first", "second", "future"]
    assert _latest_at_or_before(stamps, samples, 0.9) is None
    assert _latest_at_or_before(stamps, samples, 1.2) == (1.2, "second")
    assert _latest_at_or_before(stamps, samples, 1.39) == (1.2, "second")


def test_delayed_rtk_writes_are_kept_in_header_time_order():
    stamps, samples = [], []
    _append_time_ordered(stamps, samples, 1.2, "second")
    _append_time_ordered(stamps, samples, 1.0, "first")
    _append_time_ordered(stamps, samples, 1.4, "third")
    assert stamps == [1.0, 1.2, 1.4]
    assert samples == ["first", "second", "third"]
