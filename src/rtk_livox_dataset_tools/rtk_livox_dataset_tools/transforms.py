import math

import numpy as np


def wrap_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff_rad(a, b):
    return wrap_angle_rad(a - b)


def yaw_from_vector(v_east, v_north):
    return math.atan2(v_north, v_east)


def rotation_from_yaw(yaw_enu_lidar):
    c = math.cos(yaw_enu_lidar)
    s = math.sin(yaw_enu_lidar)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def circular_mean(angles_rad, weights=None):
    angles = np.asarray(angles_rad, dtype=float)
    if angles.size == 0:
        raise ValueError("At least one angle is required")
    if weights is None:
        weights = np.ones_like(angles)
    weights = np.asarray(weights, dtype=float)
    sin_mean = float(np.sum(weights * np.sin(angles)) / np.sum(weights))
    cos_mean = float(np.sum(weights * np.cos(angles)) / np.sum(weights))
    return math.atan2(sin_mean, cos_mean)


def circular_std(angles_rad, weights=None):
    angles = np.asarray(angles_rad, dtype=float)
    if angles.size == 0:
        return float("nan")
    if weights is None:
        weights = np.ones_like(angles)
    weights = np.asarray(weights, dtype=float)
    sin_mean = float(np.sum(weights * np.sin(angles)) / np.sum(weights))
    cos_mean = float(np.sum(weights * np.cos(angles)) / np.sum(weights))
    resultant = min(1.0, max(1.0e-12, math.hypot(sin_mean, cos_mean)))
    return math.sqrt(-2.0 * math.log(resultant))


# Backward-compatible aliases for older local code.
rotation_enu_from_lidar = rotation_from_yaw
rotation_lidar_from_enu = lambda yaw: rotation_from_yaw(yaw).T
