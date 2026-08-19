import numpy
import scipy

class ExtendedKalmanFilterXYZWH:
    def __init__(self):
        """
        기존:
            self.ndim = 7  # x, y, z, w, h, l, yaw
            self.dim_x = 3 * self.ndim  # (pos, vel, acc) -> 21
            self.dim_z = self.ndim      # 측정: [x, y, z, w, h, l, yaw]
        
        수정 후 (등속도 모델):
            self.ndim = 7
            -> 상태 벡터: [x, y, z, w, h, l, yaw,
                           vx, vy, vz, vw, vh, vl, vyaw] (총 14 차원)
        """
        self.ndim = 7  # (x, y, z, w, h, l, yaw)
        self.dt = 0.1  # 시간 간격 (10Hz)
        self.dim_x = 2 * self.ndim  # 14차원 (pos, vel)
        self.dim_z = self.ndim      # 7

        # 상태 전이 행렬 (등속도 모델: F = [I, dt*I; 0, I])
        self._motion_mat = numpy.eye(self.dim_x)
        self._motion_mat[:self.ndim, self.ndim:] = self.dt * numpy.eye(self.ndim)

        # 관측 행렬 (상태의 앞 7개 요소만 관측)
        self._update_mat = numpy.zeros((self.dim_z, self.dim_x))
        self._update_mat[:self.ndim, :self.ndim] = numpy.eye(self.ndim)

        # 표준 편차 가중치 (임의 설정)
        self._std_weight_position = 1.0 / 20
        self._std_weight_velocity = 1.0 / 160

    def f(self, state):
        """
        상태 전이 함수 (등속도 모델)
        상태 벡터 state = [pos(7), vel(7)]
        pos: [x, y, z, w, h, l, yaw]
        vel: [vx, vy, vz, vw, vh, vl, vyaw]

        등속도 모델:
            new_pos = pos + dt * vel
            new_vel = vel
        """
        dt = self.dt
        pos = state[:self.ndim]
        vel = state[self.ndim:2*self.ndim]
        new_pos = pos + dt * vel
        new_vel = vel  # 속도는 그대로 유지
        return numpy.concatenate([new_pos, new_vel])

    def f_jacobian(self, state):
        """
        f(state)의 자코비안 행렬 (14x14)
        등속도 모델의 경우:
            F = [I, dt * I; 0, I]
        """
        return self._motion_mat.copy()

    def h(self, state):
        """
        관측 함수 (선형): 상태의 앞 7개 요소만 관측
        """
        return numpy.dot(self._update_mat, state)

    def initiate(self, measurement):
        """
        측정값으로부터 초기 상태(mean)와 공분산(covariance)을 설정.
        measurement: [x, y, z, w, h, l, yaw]
        상태 벡터: [measurement, zeros(7)]
        """
        mean_pos = measurement[:self.ndim]   # (7,)
        mean_vel = numpy.zeros(self.ndim)      # (7,)
        mean = numpy.concatenate([mean_pos, mean_vel])  # (14,)
        ref_scale = measurement[3]  # width 기준 스케일
        std_pos = 2 * self._std_weight_position * ref_scale * numpy.ones(self.ndim)
        std_vel = 10 * self._std_weight_velocity * ref_scale * numpy.ones(self.ndim)
        std = numpy.concatenate([std_pos, std_vel])
        covariance = numpy.diag(numpy.square(std))
        return mean, covariance

    def predict(self, mean, covariance):
        """
        예측 단계: 현재 상태와 공분산으로부터 다음 상태 예측 (등속도 모델)
        EKF에서는 f(state)와 f_jacobian(state)를 사용.
        """
        mean_pred = self.f(mean)
        F = self.f_jacobian(mean)
        
        ref_scale = mean[3]  # width 기준 스케일
        std_pos = self._std_weight_position * ref_scale * numpy.ones(self.ndim)
        std_vel = self._std_weight_velocity * ref_scale * numpy.ones(self.ndim)
        std = numpy.concatenate([std_pos, std_vel])
        motion_cov = numpy.diag(numpy.square(std))
        
        cov_pred = F.dot(covariance).dot(F.T) + motion_cov
        return mean_pred, cov_pred

    def multi_predict(self, means, covariances):
        """
        여러 트랙에 대해 벡터화된 예측.
        각 트랙별로 f와 f_jacobian을 적용.
        """
        N = len(means)
        if N == 0:
            return means, covariances
        means_pred = []
        covs_pred = []
        for i in range(N):
            m_pred, cov_pred = self.predict(means[i], covariances[i])
            means_pred.append(m_pred)
            covs_pred.append(cov_pred)
        return numpy.stack(means_pred), numpy.stack(covs_pred)

    def update(self, mean, covariance, measurement):
        """
        업데이트 단계: 측정값 [x, y, z, w, h, l, yaw] 이용.
        EKF 업데이트: h(state)는 선형이므로 _update_mat 사용.
        """
        H = self._update_mat
        R = self._measurement_noise(mean)
        z_pred = self.h(mean)  # (7,)
        S = H.dot(covariance).dot(H.T) + R
        K = covariance.dot(H.T).dot(numpy.linalg.inv(S))
        innovation = measurement - z_pred
        new_mean = mean + K.dot(innovation)
        new_covariance = covariance - K.dot(S).dot(K.T)
        return new_mean, new_covariance

    def project(self, mean, covariance):
        """
        상태를 관측 공간으로 사영 (Projection)
        """
        z_pred = self.h(mean)
        R = self._measurement_noise(mean)
        S = self._update_mat.dot(covariance).dot(self._update_mat.T) + R
        return z_pred, S

    def _measurement_noise(self, mean):
        """
        관측 잡음 공분산 (간단 구현)
        """
        ref_scale = mean[3]
        std = 0.1 * self._std_weight_position * ref_scale * numpy.ones(self.ndim)
        return numpy.diag(numpy.square(std))
