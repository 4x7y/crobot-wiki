#pragma once

#include <limits>

namespace crobot::math {

template <typename T>
struct constants
{
  static constexpr T epsilon() {
    return 1e-6;
  }
  static constexpr T pi() {
    return 3.141592653589793238462643383279502884197169399375105820974944592L;
  }
  static constexpr T two_pi() {
    return 2.0 * pi();
  }
  static constexpr T half_pi() {
    return 0.5 * pi();
  }
  static constexpr T pi_sqr() {
    return pi() * pi();
  }

  /// The golden ratio
  static constexpr T phi() {
    return 1.618033988749894848204586834365638117720309179805762862135448623L;
  }

  static constexpr T inf() {
    return std::numeric_limits<T>::infinity();
  }
};

using constantsf = constants<float>;
using constantsd = constants<double>;

} // namespace crobot::math

