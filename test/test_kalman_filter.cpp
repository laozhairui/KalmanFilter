
#include "KalmanFilter.h"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <random>

int main()
{
    // ========================================================
    // 1. 设置模拟参数
    // ========================================================

    const double dt = 0.01;
    const int frame_count = 1000;

    // 模拟 PnP 的测量噪声标准差：5 个位置单位
    const double measurement_std = 5.0;

    // 加速度噪声方差
    const Eigen::Vector3d acceleration_variance(
        25.0, 25.0, 25.0
    );

    // 测量噪声协方差 R
    const kalman::KalmanFilter::MeasurementMatrix measurement_noise =
        measurement_std * measurement_std
        * kalman::KalmanFilter::MeasurementMatrix::Identity();

    // 创建滤波器
    kalman::KalmanFilter filter(
        acceleration_variance,
        measurement_noise
    );

    // ========================================================
    // 2. 设置真实运动状态
    // ========================================================

    const Eigen::Vector3d true_initial_position(
        100.0, 50.0, 1000.0
    );

    const Eigen::Vector3d true_velocity(
        100.0, -30.0, 20.0
    );

    // 使用固定随机种子，保证每次测试结果可复现
    std::mt19937 generator(42);

    std::normal_distribution<double> noise(
        0.0, measurement_std
    );

    // ========================================================
    // 3. 获取初始测量并初始化滤波器
    // ========================================================

    Eigen::Vector3d initial_measurement =
        true_initial_position;

    for (int axis = 0; axis < 3; ++axis)
    {
        initial_measurement(axis) += noise(generator);
    }

    // 初始速度未知，因此初始速度方差设置较大
    const Eigen::Vector3d initial_velocity =
        Eigen::Vector3d::Zero();

    const Eigen::Vector3d position_variance(
        25.0, 25.0, 25.0
    );

    const Eigen::Vector3d velocity_variance(
        10000.0, 10000.0, 10000.0
    );

    filter.initialize(
        initial_measurement,
        initial_velocity,
        position_variance,
        velocity_variance
    );

    // ========================================================
    // 4. 模拟运动、测量、预测和更新
    // ========================================================

    double raw_squared_error = 0.0;
    double filtered_squared_error = 0.0;

    for (int frame = 1; frame <= frame_count; ++frame)
    {
        const double elapsed_time = frame * dt;

        // 计算当前时刻的真实位置
        const Eigen::Vector3d true_position =
            true_initial_position
            + true_velocity * elapsed_time;

        // 模拟带有随机噪声的 PnP 测量
        Eigen::Vector3d measured_position = true_position;

        for (int axis = 0; axis < 3; ++axis)
        {
            measured_position(axis) += noise(generator);
        }

        // 先预测，再利用测量值更新
        filter.predict(dt);

        filter.update(
            measured_position
        );

        // 获取滤波后的位置
        const Eigen::Vector3d filtered_position =
            filter.get_state().head<3>();

        // 计算原始测量误差
        raw_squared_error +=
            (measured_position - true_position).squaredNorm();

        // 计算滤波结果误差
        filtered_squared_error +=
            (filtered_position - true_position).squaredNorm();

        // 每隔 100 帧输出一次结果
        if (frame % 100 == 0)
        {
            std::cout << "Frame: " << frame << '\n';

            std::cout << "True position: "
                      << true_position.transpose() << '\n';

            std::cout << "Measured position: "
                      << measured_position.transpose() << '\n';

            std::cout << "Filtered position: "
                      << filtered_position.transpose() << '\n';

            std::cout << "Estimated velocity: "
                      << filter.get_state().tail<3>().transpose()
                      << '\n';

            std::cout << "--------------------------------\n";
        }
    }

    // ========================================================
    // 5. 计算均方根误差 RMSE
    // ========================================================

    const double raw_rmse =
        std::sqrt(raw_squared_error / frame_count);

    const double filtered_rmse =
        std::sqrt(filtered_squared_error / frame_count);

    std::cout << "\n========== Test Result ==========\n";

    std::cout << "Raw measurement RMSE: "
              << raw_rmse << '\n';

    std::cout << "Filtered position RMSE: "
              << filtered_rmse << '\n';

    std::cout << "Final estimated velocity: "
              << filter.get_state().tail<3>().transpose()
              << '\n';

    std::cout << "True velocity: "
              << true_velocity.transpose()
              << '\n';

    return 0;
}
