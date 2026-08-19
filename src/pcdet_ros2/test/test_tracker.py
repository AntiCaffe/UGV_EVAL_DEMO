"""Regression tests for the standalone 3D tracker core."""

import numpy as np

from pcdet_ros2.nn_3d import BYTETracker, compute_iou_3d
from pcdet_ros2.util_3d import ExtendedKalmanFilterXYZWH


def _box(x=0.0, yaw=0.0, index=0.0):
    return np.array([[x, 0.0, 0.0, 4.0, 2.0, 2.0, yaw, index]])


def _update(tracker, timestamp, x, score=0.9, object_class=1.0):
    return tracker.update(
        _box(x=x),
        np.array([score]),
        np.array([object_class]),
        timestamp=timestamp,
    )


def test_yaw_innovation_wraps_across_pi():
    kalman_filter = ExtendedKalmanFilterXYZWH()
    mean, covariance = kalman_filter.initiate(_box(yaw=np.pi - 0.02)[0, :7])
    measurement = _box(yaw=-np.pi + 0.02)[0, :7]
    mean, _ = kalman_filter.update(mean, covariance, measurement)
    circular_error = np.arctan2(
        np.sin(mean[6] - measurement[6]),
        np.cos(mean[6] - measurement[6]),
    )
    assert abs(circular_error) < 0.04


def test_ekf_transition_jacobian_matches_finite_difference():
    kalman_filter = ExtendedKalmanFilterXYZWH(dt=0.13)
    state = np.array([
        2.0, -1.0, 0.5, 4.0, 2.0, 1.5, 0.7,
        3.0, 0.4, -0.2, 0.01, -0.02, 0.03, 0.15,
    ])
    analytical = kalman_filter.f_jacobian(state)
    numerical = np.zeros_like(analytical)
    epsilon = 1e-6
    for column in range(state.size):
        offset = np.zeros_like(state)
        offset[column] = epsilon
        numerical[:, column] = (
            kalman_filter.f(state + offset)
            - kalman_filter.f(state - offset)
        ) / (2.0 * epsilon)
    np.testing.assert_allclose(analytical, numerical, atol=1e-6)


def test_ekf_body_velocity_is_published_in_cartesian_frame():
    kalman_filter = ExtendedKalmanFilterXYZWH()
    mean, _ = kalman_filter.initiate(
        _box(yaw=np.pi / 2.0)[0, :7]
    )
    mean[7] = 2.0
    velocity = kalman_filter.velocity(mean)
    np.testing.assert_allclose(velocity, [0.0, 2.0, 0.0], atol=1e-7)


def test_iou_is_yaw_aware():
    base = _box()[0, :7]
    rotated = _box(yaw=np.pi / 2.0)[0, :7]
    assert np.isclose(compute_iou_3d([base], [base])[0, 0], 1.0)
    assert 0.2 < compute_iou_3d([base], [rotated])[0, 0] < 0.5


def test_low_score_second_association_keeps_track_id():
    tracker = BYTETracker(
        frame_rate=10,
        class_score_thresholds=[0.5, 0.3, 0.4],
    )
    first = _update(tracker, 0.0, 0.0, score=0.9)
    second = _update(tracker, 0.1, 0.1, score=0.2)
    assert first.shape == (1, 14)
    assert second.shape == (1, 14)
    assert first[0, 6] == second[0, 6]


def test_class_threshold_can_be_lower_than_global_low_threshold():
    tracker = BYTETracker(
        frame_rate=10,
        class_score_thresholds=[0.01],
        low_score_threshold=0.1,
    )
    result = _update(tracker, 0.0, 0.0, score=0.05)
    assert result.shape == (1, 14)


def test_class_mismatch_does_not_reuse_id():
    tracker = BYTETracker(
        frame_rate=10,
        class_score_thresholds=[0.5, 0.3, 0.4],
    )
    car = _update(tracker, 0.0, 0.0, object_class=1.0)
    pedestrian = _update(tracker, 0.1, 0.0, object_class=2.0)
    assert car.shape == (1, 14)
    # A new track outside the first frame requires one confirmation frame.
    assert pedestrian.shape == (0, 14)
    pedestrian = _update(tracker, 0.2, 0.0, object_class=2.0)
    assert pedestrian.shape == (1, 14)
    assert pedestrian[0, 6] != car[0, 6]


def test_lost_timeout_uses_elapsed_seconds():
    tracker = BYTETracker(
        frame_rate=10,
        class_score_thresholds=[0.5],
        max_time_lost=0.25,
    )
    _update(tracker, 0.0, 0.0)
    empty_boxes = np.empty((0, 8))
    empty = np.empty(0)
    tracker.update(empty_boxes, empty, empty, timestamp=0.1)
    assert len(tracker.lost_tracks) == 1
    tracker.update(empty_boxes, empty, empty, timestamp=0.2)
    tracker.update(empty_boxes, empty, empty, timestamp=0.3)
    assert len(tracker.lost_tracks) == 0
    assert len(tracker.removed_tracks) == 1


def test_long_sensor_gap_is_not_hidden_by_filter_dt_clamp():
    tracker = BYTETracker(
        frame_rate=10,
        class_score_thresholds=[0.5],
        max_time_lost=1.0,
    )
    _update(tracker, 0.0, 0.0)
    empty_boxes = np.empty((0, 8))
    empty = np.empty(0)
    tracker.update(empty_boxes, empty, empty, timestamp=2.0)
    assert np.isclose(tracker.kalman_filter.dt, 0.5)
    assert len(tracker.lost_tracks) == 0
    assert len(tracker.removed_tracks) == 1


def test_timestamp_changes_filter_interval_and_velocity_state():
    tracker = BYTETracker(
        frame_rate=30,
        class_score_thresholds=[0.5],
    )
    _update(tracker, 10.0, 0.0)
    result = _update(tracker, 10.2, 0.2)
    assert np.isclose(tracker.kalman_filter.dt, 0.2)
    assert result[0, 10] > 0.0
