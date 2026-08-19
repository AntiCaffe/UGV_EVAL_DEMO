"""Extended Kalman filter utilities for 3D box tracking."""

import numpy as np


def wrap_angle(angle):
    """Wrap radians to [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class ExtendedKalmanFilterXYZWH:
    """EKF-CV for a yawed 3D box.

    The state is ``[x, y, z, dx, dy, dz, yaw, v_forward, v_lateral,
    vz, vdx, vdy, vdz, yaw_rate]``.  Body-frame planar velocity makes the
    transition nonlinear with respect to yaw, so covariance prediction uses
    the state-dependent Jacobian of :meth:`f`.
    """

    def __init__(self, dt=0.1):
        self.ndim = 7
        self.dim_x = 2 * self.ndim
        self.dim_z = self.ndim
        self._update_mat = np.zeros((self.dim_z, self.dim_x), dtype=np.float64)
        self._update_mat[:, :self.ndim] = np.eye(self.ndim)
        self._std_weight_position = 1.0 / 20.0
        self._std_weight_velocity = 1.0 / 80.0
        self.set_dt(dt)

    def set_dt(self, dt):
        """Set the interval used by the nonlinear transition function."""
        self.dt = float(dt)

    def f(self, state):
        """Apply nonlinear body-frame constant-velocity motion."""
        predicted = np.asarray(state, dtype=np.float64).copy()
        yaw = state[6]
        forward_velocity = state[7]
        lateral_velocity = state[8]
        cosine, sine = np.cos(yaw), np.sin(yaw)

        predicted[0] += self.dt * (
            forward_velocity * cosine - lateral_velocity * sine
        )
        predicted[1] += self.dt * (
            forward_velocity * sine + lateral_velocity * cosine
        )
        predicted[2] += self.dt * state[9]
        predicted[3:6] += self.dt * state[10:13]
        predicted[6] = wrap_angle(state[6] + self.dt * state[13])
        return predicted

    def f_jacobian(self, state):
        """Return the state-dependent Jacobian of :meth:`f`."""
        yaw = state[6]
        forward_velocity = state[7]
        lateral_velocity = state[8]
        cosine, sine = np.cos(yaw), np.sin(yaw)
        jacobian = np.eye(self.dim_x, dtype=np.float64)

        jacobian[0, 6] = self.dt * (
            -forward_velocity * sine - lateral_velocity * cosine
        )
        jacobian[0, 7] = self.dt * cosine
        jacobian[0, 8] = -self.dt * sine
        jacobian[1, 6] = self.dt * (
            forward_velocity * cosine - lateral_velocity * sine
        )
        jacobian[1, 7] = self.dt * sine
        jacobian[1, 8] = self.dt * cosine
        jacobian[2, 9] = self.dt
        jacobian[3, 10] = self.dt
        jacobian[4, 11] = self.dt
        jacobian[5, 12] = self.dt
        jacobian[6, 13] = self.dt
        return jacobian

    @staticmethod
    def velocity(state):
        """Convert body-frame EKF velocity to Cartesian vx, vy, vz."""
        yaw = state[6]
        cosine, sine = np.cos(yaw), np.sin(yaw)
        forward_velocity = state[7]
        lateral_velocity = state[8]
        return np.array([
            forward_velocity * cosine - lateral_velocity * sine,
            forward_velocity * sine + lateral_velocity * cosine,
            state[9],
        ])

    @staticmethod
    def _scale(values):
        return max(0.1, float(np.mean(np.abs(values[3:6]))))

    def initiate(self, measurement):
        measurement = np.asarray(measurement[:self.ndim], dtype=np.float64).copy()
        measurement[6] = wrap_angle(measurement[6])
        mean = np.r_[measurement, np.zeros(self.ndim, dtype=np.float64)]
        scale = self._scale(measurement)
        std_pos = np.array([
            2.0 * self._std_weight_position * scale,
            2.0 * self._std_weight_position * scale,
            2.0 * self._std_weight_position * scale,
            0.1 * scale, 0.1 * scale, 0.1 * scale, 0.15,
        ])
        std_vel = np.array([
            0.5 * scale,
            0.5 * scale,
            0.5 * scale,
            0.05 * scale, 0.05 * scale, 0.05 * scale, 0.2,
        ])
        return mean, np.diag(np.square(np.r_[std_pos, std_vel]))

    def _motion_noise(self, mean):
        scale = self._scale(mean)
        dt_scale = max(self.dt, 1e-3) / 0.1
        std_pos = np.array([
            self._std_weight_position * scale,
            self._std_weight_position * scale,
            self._std_weight_position * scale,
            0.02 * scale, 0.02 * scale, 0.02 * scale, 0.04,
        ]) * np.sqrt(dt_scale)
        std_vel = np.array([
            4.0 * self._std_weight_velocity * scale,
            4.0 * self._std_weight_velocity * scale,
            4.0 * self._std_weight_velocity * scale,
            0.01 * scale, 0.01 * scale, 0.01 * scale, 0.04,
        ]) * np.sqrt(dt_scale)
        return np.diag(np.square(np.r_[std_pos, std_vel]))

    def _measurement_noise(self, mean):
        scale = self._scale(mean)
        std = np.array([
            0.5 * self._std_weight_position * scale,
            0.5 * self._std_weight_position * scale,
            0.5 * self._std_weight_position * scale,
            0.04 * scale, 0.04 * scale, 0.04 * scale, 0.08,
        ])
        return np.diag(np.square(std))

    def predict(self, mean, covariance):
        transition_jacobian = self.f_jacobian(mean)
        mean = self.f(mean)
        covariance = (
            transition_jacobian @ covariance @ transition_jacobian.T
            + self._motion_noise(mean)
        )
        return mean, covariance

    def multi_predict(self, means, covariances):
        if len(means) == 0:
            return means, covariances
        predictions = [
            self.predict(mean, covariance)
            for mean, covariance in zip(means, covariances)
        ]
        return (
            np.stack([item[0] for item in predictions]),
            np.stack([item[1] for item in predictions]),
        )

    def project(self, mean, covariance):
        projected_mean = self._update_mat @ mean
        projected_mean[6] = wrap_angle(projected_mean[6])
        projected_covariance = (
            self._update_mat @ covariance @ self._update_mat.T
            + self._measurement_noise(mean)
        )
        return projected_mean, projected_covariance

    def update(self, mean, covariance, measurement):
        measurement = np.asarray(measurement[:self.ndim], dtype=np.float64)
        projected_mean, projected_covariance = self.project(mean, covariance)
        innovation = measurement - projected_mean
        innovation[6] = wrap_angle(innovation[6])
        cross_covariance = covariance @ self._update_mat.T
        kalman_gain = np.linalg.solve(
            projected_covariance, cross_covariance.T
        ).T
        new_mean = mean + kalman_gain @ innovation
        new_mean[6] = wrap_angle(new_mean[6])

        identity = np.eye(self.dim_x)
        residual = identity - kalman_gain @ self._update_mat
        measurement_covariance = self._measurement_noise(mean)
        new_covariance = (
            residual @ covariance @ residual.T
            + kalman_gain @ measurement_covariance @ kalman_gain.T
        )
        new_covariance = 0.5 * (new_covariance + new_covariance.T)
        return new_mean, new_covariance

    def gating_distance(self, mean, covariance, measurements):
        """Squared Mahalanobis distance using measured 3D centers."""
        measurements = np.asarray(measurements, dtype=np.float64)
        if measurements.size == 0:
            return np.empty(0, dtype=np.float64)
        projected_mean, projected_covariance = self.project(mean, covariance)
        delta = measurements[:, :3] - projected_mean[None, :3]
        center_covariance = projected_covariance[:3, :3]
        try:
            solved = np.linalg.solve(center_covariance, delta.T).T
        except np.linalg.LinAlgError:
            solved = delta @ np.linalg.pinv(center_covariance)
        return np.einsum('ij,ij->i', delta, solved)
