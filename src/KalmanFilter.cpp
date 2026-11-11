
#include "KalmanFilter.h"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace kalman
{

KalmanFilter::KalmanFilter(
    const Eigen::Vector3d& acceleration_variance,
    const MeasurementMatrix& measurement_noise
)
    : m_state(StateVector::Zero()),
      m_covariance(StateMatrix::Zero()),
      m_acceleration_variance(acceleration_variance),
      m_measurement_noise(measurement_noise)
{
    // 检查加速度噪声方差是否合法
    if (!m_acceleration_variance.allFinite()
        || (m_acceleration_variance.array() < 0.0).any())
    {
        throw std::invalid_argument(
            "Acceleration variance must be finite and non-negative"
        );
    }

    // 检查测量噪声矩阵是否包含非法数值
    if (!m_measurement_noise.allFinite())
    {
        throw std::invalid_argument(
            "Measurement noise must contain finite values"
        );
    }

    // R 应当是对称矩阵
    if (!m_measurement_noise.isApprox(
            m_measurement_noise.transpose(), 1e-10))
    {
        throw std::invalid_argument(
            "Measurement noise matrix must be symmetric"
        );
    }

    // R 应当是正定矩阵
    Eigen::LLT<MeasurementMatrix> llt(m_measurement_noise);

    if (llt.info() != Eigen::Success)
    {
        throw std::invalid_argument(
            "Measurement noise matrix must be positive definite"
        );
    }
}


void KalmanFilter::initialize(
    const Eigen::Vector3d& initial_position,
    const Eigen::Vector3d& initial_velocity,
    const Eigen::Vector3d& position_variance,
    const Eigen::Vector3d& velocity_variance
)
{
    // 检查初始状态和方差是否合法
    if (!initial_position.allFinite()
        || !initial_velocity.allFinite()
        || !position_variance.allFinite()
        || !velocity_variance.allFinite())
    {
        throw std::invalid_argument(
            "Initial state and variances must be finite"
        );
    }

    if ((position_variance.array() < 0.0).any()
        || (velocity_variance.array() < 0.0).any())
    {
        throw std::invalid_argument(
            "Initial variances must be non-negative"
        );
    }

    // 初始化状态向量
    m_state.head<3>() = initial_position;
    m_state.tail<3>() = initial_velocity;

    // 初始化状态协方差矩阵 P
    m_covariance.setZero();

    m_covariance.diagonal().head<3>() = position_variance;
    m_covariance.diagonal().tail<3>() = velocity_variance;

    m_is_initialized = true;
}


void KalmanFilter::predict(double dt)
{
    // 必须先初始化滤波器
    if (!m_is_initialized)
    {
        throw std::logic_error(
            "KalmanFilter must be initialized before prediction"
        );
    }

    // 检查时间间隔是否合法
    if (!std::isfinite(dt) || dt <= 0.0)
    {
        throw std::invalid_argument(
            "dt must be finite and positive"
        );
    }

    // ========================================================
    // 1. 构建状态转移矩阵 F
    // ========================================================

    TransitionMatrix transition_matrix =
        TransitionMatrix::Identity();

    transition_matrix(0, 3) = dt;
    transition_matrix(1, 4) = dt;
    transition_matrix(2, 5) = dt;

    // ========================================================
    // 2. 构建噪声映射矩阵 G
    // ========================================================

    NoiseMappingMatrix noise_mapping_matrix =
        NoiseMappingMatrix::Zero();

    const double half_dt_squared = 0.5 * dt * dt;

    noise_mapping_matrix(0, 0) = half_dt_squared;
    noise_mapping_matrix(1, 1) = half_dt_squared;
    noise_mapping_matrix(2, 2) = half_dt_squared;

    noise_mapping_matrix(3, 0) = dt;
    noise_mapping_matrix(4, 1) = dt;
    noise_mapping_matrix(5, 2) = dt;

    // ========================================================
    // 3. 构建加速度噪声协方差矩阵 Σa
    // ========================================================

    Eigen::Matrix3d acceleration_noise =
        m_acceleration_variance.asDiagonal();

    // ========================================================
    // 4. 计算过程噪声协方差矩阵 Q
    // ========================================================

    StateMatrix process_noise =
        noise_mapping_matrix
        * acceleration_noise
        * noise_mapping_matrix.transpose();

    // ========================================================
    // 5. 预测状态向量
    // ========================================================

    m_state = transition_matrix * m_state;

    // ========================================================
    // 6. 预测状态协方差矩阵
    // ========================================================

    m_covariance =
        transition_matrix
        * m_covariance
        * transition_matrix.transpose()
        + process_noise;

    // 减少浮点计算导致的非对称误差
    m_covariance =
        0.5 * (m_covariance + m_covariance.transpose());
}


void KalmanFilter::update(const MeasurementVector& measurement)
{
    // 必须先初始化滤波器
    if (!m_is_initialized)
    {
        throw std::logic_error(
            "KalmanFilter must be initialized before update"
        );
    }

    if (!measurement.allFinite())
    {
        throw std::invalid_argument(
            "Measurement must contain finite values"
        );
    }

    // ========================================================
    // 1. 构建观测矩阵 H
    // ========================================================

    Eigen::Matrix<double, 3, 6> observation_matrix =
        Eigen::Matrix<double, 3, 6>::Zero();

    observation_matrix(0, 0) = 1.0;
    observation_matrix(1, 1) = 1.0;
    observation_matrix(2, 2) = 1.0;

    // ========================================================
    // 2. 计算测量残差（Innovation）
    // ========================================================

    const MeasurementVector innovation =
        measurement - observation_matrix * m_state;

    // ========================================================
    // 3. 计算残差协方差矩阵 S
    // ========================================================

    const MeasurementMatrix innovation_covariance =
        observation_matrix
        * m_covariance
        * observation_matrix.transpose()
        + m_measurement_noise;

    // ========================================================
    // 4. 使用 LDLT 分解计算卡尔曼增益 K
    // ========================================================

    Eigen::LDLT<MeasurementMatrix> ldlt(
        innovation_covariance
    );

    if (ldlt.info() != Eigen::Success)
    {
        throw std::runtime_error(
            "Failed to decompose innovation covariance"
        );
    }

    const Eigen::Matrix<double, 6, 3> covariance_measurement =
        m_covariance * observation_matrix.transpose();

    const Eigen::Matrix<double, 6, 3> kalman_gain =
        ldlt.solve(covariance_measurement.transpose()).transpose();

    // ========================================================
    // 5. 更新状态向量
    // ========================================================

    m_state += kalman_gain * innovation;

    // ========================================================
    // 6. 使用 Joseph 形式更新状态协方差矩阵
    // ========================================================

    const StateMatrix identity_matrix = StateMatrix::Identity();

    const StateMatrix correction_matrix =
        identity_matrix - kalman_gain * observation_matrix;

    m_covariance =
        correction_matrix
        * m_covariance
        * correction_matrix.transpose()
        + kalman_gain
        * m_measurement_noise
        * kalman_gain.transpose();

    // 减少浮点计算导致的非对称误差
    m_covariance =
        0.5 * (m_covariance + m_covariance.transpose());
}


const KalmanFilter::StateVector& KalmanFilter::get_state() const
{
    return m_state;
}


const KalmanFilter::StateMatrix& KalmanFilter::get_covariance() const
{
    return m_covariance;
}


bool KalmanFilter::is_initialized() const noexcept
{
    return m_is_initialized;
}

}
