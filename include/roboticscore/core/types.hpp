#pragma once

/// @file types.hpp
/// @brief Short aliases for the Eigen types used throughout the library.

#include <Eigen/Core>

namespace roboticscore {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecX = Eigen::VectorXd;

using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using MatX = Eigen::MatrixXd;

/// Index type used by Eigen for sizes and coefficients.
using Index = Eigen::Index;

}  // namespace roboticscore
