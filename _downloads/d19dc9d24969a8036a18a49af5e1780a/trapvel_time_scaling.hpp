#pragma once

#include "crobot/planning/time_scaling.hpp"

namespace crobot::planning {

class TrapvelTimeScaling : public TimeScaling {
public:
  using Ptr      = std::shared_ptr<TrapvelTimeScaling>;
  using ConstPtr = std::shared_ptr<const TrapvelTimeScaling>;

  explicit TrapvelTimeScaling(double duration);

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

