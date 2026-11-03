#pragma once

#include <vector>

namespace crobot::math {

/**
 * @brief Returns the mulitplicity of the knot at index
 *
 * @param[in] knots Knot vector
 * @param[in] index Index of knot of interest
 * @return Multiplicity (>= 1)
 */
int knot_multiplicity(const std::vector<double>& knots, int index);

} // namespace crobot::math

#include "crobot/math/detail/spline_impl.hpp"

