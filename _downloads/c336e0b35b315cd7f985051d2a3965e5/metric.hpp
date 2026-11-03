#pragma once

#include "crobot/math/state_space.hpp"

namespace crobot::math {

template <typename T>
class Metric {
public:
  explicit Metric() = default;

  virtual ~Metric() = default;

  virtual double distance(const T& from, const T& to) const = 0;

  virtual double distance(double from, double to, int index) const {
    (void)index;
    return std::abs(from - to);
  }
};

} // namespace crobot::math

