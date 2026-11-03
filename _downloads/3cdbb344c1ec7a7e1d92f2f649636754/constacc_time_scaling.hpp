#pragma once

#include "crobot/planning/time_scaling.hpp"

namespace crobot::planning {

class ConstaccTimeScaling : public TimeScaling {
public:
  using Ptr      = std::shared_ptr<ConstaccTimeScaling>;
  using ConstPtr = std::shared_ptr<const ConstaccTimeScaling>;

  /**
   * @brief Construct a new Time Scaling object with constant acceleration
   *
   * @param acc The acceleration
   */
  explicit ConstaccTimeScaling(double acc);

  // Documentation inherited
  double get_scaling_factor(double time) const override;

  // Documentation inherited
  double get_scaling_factor_deriv(double time) const override;

  // Documentation inherited
  double get_scaling_factor_deriv2(double time) const override;

private:
  double m_acc;
};

} // namespace crobot::planning

