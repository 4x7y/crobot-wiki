#pragma once

#include <cmath>

#include "crobot/math/constants.hpp"

namespace crobot::math {

/**
 * Check if two numbers are close enough within eps
 * @param[in] a First number
 * @param[in] b Second number
 * @param[in] eps Tolerance for checking closeness
 * @return Whether the numbers are close w.r.t. the tolerance
 */
template <typename T>
inline bool is_equal(T a, T b, double eps = std::numeric_limits<T>::epsilon()) {
  return std::abs(a - b) < eps * std::abs(a);
}
/**
 * Check if the input number is close enough to zero within eps
 * @param[in] a The input number
 * @param[in] eps Tolerance for checking closeness
 * @return Whether the number is close to zero w.r.t. the tolerance
 */
template <typename T>
inline bool is_zero(T x) {
  return std::abs(x) <= std::numeric_limits<T>::epsilon() * std::abs(x);
}

} // namespace crobot::math

