#pragma once

#include "crobot/math/math.hpp"

namespace crobot::planning {

class TimeScaling {
public:
  TimeScaling(double duration);

  virtual ~TimeScaling() = default;

  virtual double get_duration() const;

  virtual double get_scaling_factor(double time) const = 0;

  virtual double get_scaling_factor_deriv(double time) const = 0;

  virtual double get_scaling_factor_deriv2(double time) const = 0;

protected:
  double m_duration;
};

/**
 * @brief Construct linear time scaling: s(t) = (t - t0)/(tF - t0)
 */
std::vector<std::vector<double>> construct_linear_time_scaling(
    const std::pair<double, double>& time_interval,
    const std::vector<double>& time);

} // namespace crobot::planning

