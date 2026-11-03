#pragma once

#include "crobot/planning/time_scaling.hpp"

namespace crobot::planning {

class LinearTimeScaling : public TimeScaling {
public:
  LinearTimeScaling(double time_start, double time_end);

  explicit LinearTimeScaling(double duration);

  // Documentation inherited
  double get_scaling_factor(double time) const override;

  // Documentation inherited
  double get_scaling_factor_deriv(double time) const override;

  // Documentation inherited
  double get_scaling_factor_deriv2(double time) const override;

private:
  double m_scale;

  std::pair<double, double> m_time_interval;
};

} // namespace crobot::planning

