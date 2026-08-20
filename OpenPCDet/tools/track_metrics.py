"""Reusable position and velocity metrics for RTK tracking evaluation."""

import numpy as np


AXIS_NAMES = ("x", "y", "z")


def _as_xyz_matrix(name, values):
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.ndim != 2 or matrix.shape[1] != 3:
        raise ValueError("%s must have shape (N, 3)" % name)
    if len(matrix) == 0:
        raise ValueError("%s must contain at least one sample" % name)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("%s contains NaN or inf" % name)
    return matrix


def _check_same_length(named_arrays):
    lengths = {name: len(values) for name, values in named_arrays}
    if len(set(lengths.values())) != 1:
        details = ", ".join(
            "%s=%d" % (name, length) for name, length in lengths.items()
        )
        raise ValueError("Metric inputs must have equal lengths: " + details)


def _vector_errors(predicted, ground_truth, dimensions):
    if dimensions not in (2, 3):
        raise ValueError("dimensions must be 2 or 3")
    predicted = _as_xyz_matrix("predicted", predicted)
    ground_truth = _as_xyz_matrix("ground_truth", ground_truth)
    _check_same_length(
        (("predicted", predicted), ("ground_truth", ground_truth))
    )
    return predicted[:, :dimensions] - ground_truth[:, :dimensions]


def _scalar_errors(predicted, ground_truth):
    predicted = np.asarray(predicted, dtype=np.float64).reshape(-1)
    ground_truth = np.asarray(ground_truth, dtype=np.float64).reshape(-1)
    _check_same_length(
        (("predicted", predicted), ("ground_truth", ground_truth))
    )
    if len(predicted) == 0:
        raise ValueError("Scalar metric inputs must not be empty")
    if not np.all(np.isfinite(predicted)) or not np.all(
        np.isfinite(ground_truth)
    ):
        raise ValueError("Scalar metric inputs contain NaN or inf")
    return predicted - ground_truth


def calculate_vector_rmse(predicted, ground_truth, dimensions=3):
    """Calculate RMSE of Euclidean vector-error magnitudes."""
    errors = _vector_errors(predicted, ground_truth, dimensions)
    squared_distances = np.sum(np.square(errors), axis=1)
    return float(np.sqrt(np.mean(squared_distances)))


def calculate_vector_mae(predicted, ground_truth, dimensions=3):
    """Calculate mean absolute Euclidean vector error (mean distance)."""
    errors = _vector_errors(predicted, ground_truth, dimensions)
    return float(np.mean(np.linalg.norm(errors, axis=1)))


def calculate_scalar_rmse(predicted, ground_truth):
    """Calculate RMSE for scalar observations such as speed."""
    errors = _scalar_errors(predicted, ground_truth)
    return float(np.sqrt(np.mean(np.square(errors))))


def calculate_scalar_mae(predicted, ground_truth):
    """Calculate MAE for scalar observations such as speed."""
    errors = _scalar_errors(predicted, ground_truth)
    return float(np.mean(np.abs(errors)))


def vector_error_metrics(predicted, ground_truth, dimensions, unit_suffix):
    """Return Euclidean and per-axis RMSE/MAE for 2D or 3D vectors."""
    errors = _vector_errors(predicted, ground_truth, dimensions)
    axis_rmse = np.sqrt(np.mean(np.square(errors), axis=0))
    axis_mae = np.mean(np.abs(errors), axis=0)
    return {
        "rmse_" + unit_suffix: calculate_vector_rmse(
            predicted, ground_truth, dimensions
        ),
        "mae_" + unit_suffix: calculate_vector_mae(
            predicted, ground_truth, dimensions
        ),
        "axis_rmse_" + unit_suffix: {
            axis: float(value)
            for axis, value in zip(AXIS_NAMES[:dimensions], axis_rmse)
        },
        "axis_mae_" + unit_suffix: {
            axis: float(value)
            for axis, value in zip(AXIS_NAMES[:dimensions], axis_mae)
        },
    }


def scalar_error_metrics(predicted, ground_truth, unit_suffix):
    """Return RMSE, MAE and signed bias for scalar observations."""
    errors = _scalar_errors(predicted, ground_truth)
    return {
        "rmse_" + unit_suffix: calculate_scalar_rmse(
            predicted, ground_truth
        ),
        "mae_" + unit_suffix: calculate_scalar_mae(predicted, ground_truth),
        "bias_" + unit_suffix: float(np.mean(errors)),
    }


def calculate_tracking_metrics(
    predicted_positions,
    ground_truth_positions,
    predicted_velocities,
    ground_truth_velocities,
):
    """Calculate 2D/3D position, velocity-vector and scalar-speed errors."""
    predicted_positions = _as_xyz_matrix(
        "predicted_positions", predicted_positions
    )
    ground_truth_positions = _as_xyz_matrix(
        "ground_truth_positions", ground_truth_positions
    )
    predicted_velocities = _as_xyz_matrix(
        "predicted_velocities", predicted_velocities
    )
    ground_truth_velocities = _as_xyz_matrix(
        "ground_truth_velocities", ground_truth_velocities
    )
    _check_same_length(
        (
            ("predicted_positions", predicted_positions),
            ("ground_truth_positions", ground_truth_positions),
            ("predicted_velocities", predicted_velocities),
            ("ground_truth_velocities", ground_truth_velocities),
        )
    )

    predicted_speeds = np.linalg.norm(predicted_velocities, axis=1)
    ground_truth_speeds = np.linalg.norm(ground_truth_velocities, axis=1)
    return {
        "position_2d": vector_error_metrics(
            predicted_positions, ground_truth_positions, 2, "m"
        ),
        "position_3d": vector_error_metrics(
            predicted_positions, ground_truth_positions, 3, "m"
        ),
        "velocity_2d": vector_error_metrics(
            predicted_velocities, ground_truth_velocities, 2, "m_s"
        ),
        "velocity_3d": vector_error_metrics(
            predicted_velocities, ground_truth_velocities, 3, "m_s"
        ),
        "speed": scalar_error_metrics(
            predicted_speeds, ground_truth_speeds, "m_s"
        ),
    }
