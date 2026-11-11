
#pragma once

#include <Eigen/Core>

namespace kalman
{

class KalmanFilter
{
public:
    // 状态向量：[X, Y, Z, Vx, Vy, Vz]^T
    using StateVector = Eigen::Matrix<double, 6, 1>;

    // 测量向量：[Xm, Ym, Zm]^T
    using MeasurementVector = Eigen::Matrix<double, 3, 1>;

    // 状态协方差矩阵 P：6×6
    using StateMatrix = Eigen::Matrix<double, 6, 6>;

    // 测量噪声协方差矩阵 R：3×3
    using MeasurementMatrix = Eigen::Matrix<double, 3, 3>;

    // 状态转移矩阵 F：6×6
    using TransitionMatrix = Eigen::Matrix<double, 6, 6>;

    // 噪声映射矩阵 G：6×3
    using NoiseMappingMatrix = Eigen::Matrix<double, 6, 3>;

    KalmanFilter(
        const Eigen::Vector3d& acceleration_variance,
        const MeasurementMatrix& measurement_noise
    );

    void initialize(
        const Eigen::Vector3d& initial_position,
        const Eigen::Vector3d& initial_velocity,
        const Eigen::Vector3d& position_variance,
        const Eigen::Vector3d& velocity_variance
    );

    void predict(double dt);

    void update(const MeasurementVector& measurement);

    const StateVector& get_state() const;

    const StateMatrix& get_covariance() const;

    bool is_initialized() const noexcept;

private:
    StateVector m_state;

    StateMatrix m_covariance;

    // 三个方向的加速度噪声方差
    Eigen::Vector3d m_acceleration_variance;

    // 测量噪声协方差矩阵 R
    MeasurementMatrix m_measurement_noise;

    bool m_is_initialized = false;
};

}
