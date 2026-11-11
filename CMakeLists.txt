
cmake_minimum_required(VERSION 3.16)

project(KalmanFilter
    VERSION 1.0.0
    DESCRIPTION "A reusable 3D linear Kalman filter library"
    LANGUAGES CXX
)

# ============================================================
# 1. 设置 C++ 标准
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# 2. 查找 Eigen3
# ============================================================

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ============================================================
# 3. 创建卡尔曼滤波器库
# ============================================================

add_library(kalman_filter
    src/KalmanFilter.cpp
)

# 对外公开头文件目录
target_include_directories(kalman_filter
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 链接 Eigen3
target_link_libraries(kalman_filter
    PUBLIC
        Eigen3::Eigen
)

# ============================================================
# 4. 编译选项
# ============================================================

target_compile_options(kalman_filter
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# ============================================================
# 5. 构建测试程序
# ============================================================

enable_testing()

add_executable(test_kalman_filter
    test/test_kalman_filter.cpp
)

target_link_libraries(test_kalman_filter
    PRIVATE
        kalman_filter
)

add_test(
    NAME kalman_filter_test
    COMMAND test_kalman_filter
)
