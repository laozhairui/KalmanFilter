# 三维线性卡尔曼滤波器（KalmanFilter）

基于 **C++17** 和 **Eigen3** 实现的轻量级三维线性卡尔曼滤波库。

本项目实现了一个用于三维空间目标运动状态估计的线性卡尔曼滤波器，通过连续的位置测量，对目标的三维位置和速度进行估计。

适用于 RoboMaster 视觉系统中，结合 PnP 输出的三维坐标，对装甲板等运动目标进行轨迹估计与位置平滑。

## 1. 功能特性

- 使用六维状态向量，同时估计目标的三维位置和三维速度。
- 使用恒定速度（Constant Velocity）运动模型。
- 使用随机加速度描述运动过程中的不确定性。
- 支持三维位置测量输入，可直接接收 PnP 输出的 XYZ 坐标。
- 使用 Eigen3 进行矩阵运算。
- 使用 LDLT 分解计算卡尔曼增益，避免显式计算矩阵逆。
- 使用 Joseph 形式更新状态协方差矩阵，提高数值稳定性。
- 提供三维运动模拟测试，验证滤波效果。

## 2. 算法模型

### 2.1 状态向量

滤波器使用六维状态向量：

\[
\mathbf{x}=
\begin{bmatrix}
X & Y & Z & V_x & V_y & V_z
\end{bmatrix}^{T}
\]

其中：

- \(X,Y,Z\)：目标在三维空间中的位置。
- \(V_x,V_y,V_z\)：目标在三个方向上的速度。

### 2.2 测量向量

当前版本仅使用三维位置作为测量输入：

\[
\mathbf{z}=
\begin{bmatrix}
X_m & Y_m & Z_m
\end{bmatrix}^{T}
\]

测量数据可以来自 OpenCV PnP 姿态解算得到的目标平移坐标。

### 2.3 预测与更新

滤波器主要包含两个步骤：

**预测（Predict）**

\[
\mathbf{x}^{-}=F\mathbf{x}
\]

\[
P^{-}=FPF^{T}+Q
\]

根据恒定速度模型预测下一时刻的状态，并传播状态估计的不确定性。

**更新（Update）**

\[
K=P^{-}H^{T}(HP^{-}H^{T}+R)^{-1}
\]

\[
\mathbf{x}=\mathbf{x}^{-}+K(\mathbf{z}-H\mathbf{x}^{-})
\]

利用新的位置测量修正预测结果。

协方差矩阵采用 Joseph 形式更新，以提高数值稳定性。

## 3. 工程目录

```text
KalmanFilter/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/
│   └── KalmanFilter.h
├── src/
│   └── KalmanFilter.cpp
└── test/
    └── test_kalman_filter.cpp
```

目录说明：

| 目录或文件 | 作用 |
|---|---|
| `include/` | 对外公开的头文件 |
| `src/` | 卡尔曼滤波算法实现 |
| `test/` | 独立测试程序 |
| `CMakeLists.txt` | CMake 工程配置 |
| `.gitignore` | Git 忽略文件配置 |

## 4. 环境依赖

- C++17
- CMake 3.16+
- Eigen3 3.3+

Ubuntu 安装 Eigen3：

```bash
sudo apt update
sudo apt install libeigen3-dev
```

## 5. 编译与运行

克隆项目：

```bash
git clone <你的 GitHub 仓库地址>
cd KalmanFilter
```

生成构建文件：

```bash
cmake -S . -B build
```

编译：

```bash
cmake --build build -j$(nproc)
```

运行测试程序：

```bash
./build/test_kalman_filter
```

也可以使用 CTest 运行自动化测试：

```bash
ctest --test-dir build --output-on-failure
```

## 6. 接口使用

### 6.1 创建滤波器

```cpp
#include "KalmanFilter.h"

const Eigen::Vector3d acceleration_variance(
    25.0, 25.0, 25.0
);

const kalman::KalmanFilter::MeasurementMatrix measurement_noise =
    25.0 * kalman::KalmanFilter::MeasurementMatrix::Identity();

kalman::KalmanFilter filter(
    acceleration_variance,
    measurement_noise
);
```

### 6.2 初始化滤波器

```cpp
const Eigen::Vector3d initial_position(
    0.0, 0.0, 1000.0
);

const Eigen::Vector3d initial_velocity(
    0.0, 0.0, 0.0
);

const Eigen::Vector3d position_variance(
    25.0, 25.0, 25.0
);

const Eigen::Vector3d velocity_variance(
    10000.0, 10000.0, 10000.0
);

filter.initialize(
    initial_position,
    initial_velocity,
    position_variance,
    velocity_variance
);
```

### 6.3 预测与更新

每次获得新的位置测量后，先预测，再更新：

```cpp
const double dt = 0.01;

// 预测下一时刻状态
filter.predict(dt);

// 输入新的三维位置测量
const kalman::KalmanFilter::MeasurementVector measurement(
    10.0, 5.0, 1000.0
);

// 根据测量值修正预测结果
filter.update(measurement);

// 获取滤波后的六维状态
const auto& state = filter.get_state();

// 提取位置和速度
const Eigen::Vector3d position = state.head<3>();
const Eigen::Vector3d velocity = state.tail<3>();
```

其中：

- `predict(dt)`：根据运动模型预测状态。
- `update(measurement)`：利用新的测量数据修正预测结果。
- `get_state()`：获取当前六维状态向量。
- `get_covariance()`：获取当前状态协方差矩阵。
- `is_initialized()`：判断滤波器是否完成初始化。

注意：`dt` 使用秒作为单位，应根据相邻两次预测之间的实际时间间隔计算。

## 7. 参数说明

| 参数 | 含义 | 单位 |
|---|---|---|
| `acceleration_variance` | XYZ 三个方向的加速度噪声方差 | (位置单位/s²)² |
| `measurement_noise` | 测量噪声协方差矩阵 R | 位置单位² |
| `initial_position` | 初始三维位置 | 位置单位 |
| `initial_velocity` | 初始三维速度 | 位置单位/s |
| `position_variance` | 初始位置方差 | 位置单位² |
| `velocity_variance` | 初始速度方差 | (位置单位/s)² |
| `dt` | 两次预测之间的时间间隔 | s |

注意：上述噪声参数传入的是方差或协方差，而不是标准差。

参数调整原则：

- 增大 `acceleration_variance`：允许目标运动更容易偏离恒定速度模型，提高对加速度变化的响应能力。
- 减小 `acceleration_variance`：更加相信恒定速度模型。
- 增大 `measurement_noise`：降低对测量数据的信任程度，更依赖运动模型预测。
- 减小 `measurement_noise`：提高对测量数据的信任程度。
- 增大 `velocity_variance`：表示对初始速度的估计更加不确定。

所有位置和速度单位必须保持一致。例如位置使用毫米时，速度应使用毫米每秒。

## 8. 测试结果

使用模拟三维匀速运动数据进行测试：

- 真实运动速度：\((100,-30,20)\)
- 位置测量噪声：标准差为 5
- 采样时间间隔：0.01 s
- 模拟帧数：1000

测试结果：

| 指标 | 测试结果 |
|---|---:|
| 原始测量位置 RMSE | 8.82955 |
| 滤波后位置 RMSE | 1.40844 |
| 位置误差降低比例 | 约 84.1% |

最终速度估计：

| 方向 | 真实速度 | 估计速度 |
|---|---:|---:|
| X | 100 | 100.331 |
| Y | -30 | -30.0624 |
| Z | 20 | 19.9524 |

测试表明，在本次恒定速度与高斯测量噪声模拟条件下，滤波器能够有效降低位置测量误差，并估计目标的三维速度。

## 9. 当前版本的适用范围

当前版本使用三维线性恒定速度模型，适合近似匀速运动目标的状态估计。

需要注意：

- 当前版本要求每次更新都提供完整的 XYZ 位置测量。
- 当前版本没有直接估计目标加速度。
- 面对突然加速、急转弯等运动情况，需要进一步调整过程噪声，或者使用更合适的运动模型。
- 测试结果来自模拟数据，不代表真实相机和 PnP 系统中的实际精度。

## 10. 后续计划

- [ ] 支持根据实际测量时间戳计算 `dt`。
- [ ] 在真实 PnP 输出数据上测试滤波效果。
- [ ] 根据实际目标运动特性调整过程噪声。
- [ ] 探索扩展卡尔曼滤波（EKF）等更复杂的状态估计方法。
