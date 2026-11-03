#include "crobot/math/spline.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
bool crobot::math::internal::is_knot_vector_monotonic(
    const std::vector<double>& knots) {
  return std::is_sorted(knots.begin(), knots.end());
}

//==============================================================================
bool crobot::math::internal::is_knot_vector_closed(
    int degree, const std::vector<double>& knots) {
  for (int i = 0; i < degree - 1; ++i) {
    int j = knots.size() - degree + i;
    if (std::abs((knots[i + 1] - knots[i]) - (knots[j + 1] - knots[j]))
        > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  return true;
}

//==============================================================================
int crobot::math::knot_multiplicity(
    const std::vector<double>& knots, int index) {
  double u   = knots[index];
  double eps = std::numeric_limits<double>::epsilon();
  int mult   = 0;
  for (std::size_t i = index; i < knots.size(); ++i) {
    if (std::abs(u - knots[index + 1]) < eps) {
      ++mult;
    }
  }
  return mult;
}
