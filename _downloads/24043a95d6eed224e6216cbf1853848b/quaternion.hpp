#pragma once


// clang-format off
#include "crobot/math/detail/eigen_addons.hpp"
#ifndef EIGEN_QUATERNIONBASE_PLUGIN
#  define EIGEN_QUATERNIONBASE_PLUGIN "crobot/math/detail/quaternionbase_addons.hpp"
#endif
#include "crobot/math/matrix.hpp"
#include <Eigen/Geometry>
// clang-format on

#include "crobot/math/rotation.hpp"

namespace crobot::math {

using Quaternion = ::Eigen::Quaternion<double>;
} // namespace crobot::math

