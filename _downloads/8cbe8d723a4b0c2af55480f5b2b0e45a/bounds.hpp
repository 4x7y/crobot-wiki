
#pragma once

#include <vector>

namespace crobot::optimization {

/**
 * @brief Upper and lower bound for optimization variables and constraints.
 */
struct Bounds
{
  /**
   * @brief Creates a bound between @a lower and @a upper.
   */
  Bounds(double lower = 0.0, double upper = 0.0) {
    this->lower = lower;
    this->upper = upper;
  }

  double lower;
  double upper;

  void operator+=(double scalar) {
    lower += scalar;
    upper += scalar;
  }

  void operator-=(double scalar) {
    lower -= scalar;
    upper -= scalar;
  }
};

// settings this as signals infinity for IPOPT/SNOPT solvers
static const double inf = 1.0e20;

static const Bounds NoBound          = Bounds(-inf, +inf);
static const Bounds BoundZero        = Bounds(0.0, 0.0);
static const Bounds BoundGreaterZero = Bounds(0.0, +inf);
static const Bounds BoundSmallerZero = Bounds(-inf, 0.0);

using VecBound = std::vector<Bounds>;

} // namespace crobot::optimization

