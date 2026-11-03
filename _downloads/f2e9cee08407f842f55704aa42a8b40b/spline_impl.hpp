#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::math::internal {

/**
 * @brief Checks if the relation between degree, number of knots, and number
 * of control points is valid.
 *
 * @param[in] degree Degree
 * @param[in] num_knots Number of knot values
 * @param[in] num_ctrl_pts Number of control points
 * @return Whether the relationship is valid
 */
inline bool is_valid_relation(
    int degree, size_t num_knots, size_t num_ctrl_pts) {
  return (num_knots - degree - 1) == num_ctrl_pts;
}

/**
 * @brief Returns whether the knots are in ascending order.
 *
 * @param[in] knots Knot vector
 * @return Whether monotonic
 */
bool is_knot_vector_monotonic(const std::vector<double>& knots);

/**
 * @brief Returns whether the given knot vector is closed by checking the
 * periodicity of knot vectors near the start and end
 *
 * @param[in] degree Degree of curve/surface
 * @param[in] knots Knot vector of curve/surface
 * @return Whether knot vector is closed
 */
bool is_knot_vector_closed(int degree, const std::vector<double>& knots);

inline double length(const Eigen::Ref<const math::VectorXd> vec) {
  return vec.norm();
}

inline double length(double v) {
  return std::abs(v);
}

/**
 * @brief Compute the binomial coefficient (nCk) using the formula
 *
 * \product_{i=0}^k (n + 1 - i) / i
 */
inline int binomial(int n, int k) {
  int result = 1;
  if (k > n) {
    return 0;
  }
  for (int i = 1; i <= k; ++i) {
    result *= (n + 1 - i);
    result /= i;
  }
  return result;
}

} // namespace crobot::math::internal

