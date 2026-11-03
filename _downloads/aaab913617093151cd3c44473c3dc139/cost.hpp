#pragma once

#include "crobot/ik/constraint.hpp"

namespace crobot::ik {

class Cost : public ik::Constraint {
public:
  Cost(const std::string& name, const std::vector<std::string>& variables);

private:
  /**
   * @brief  Returns the scalar cost term calculated from the @c variables.
   */
  virtual double get_cost(const math::VectorXd& x) const = 0;

  /**
   * @brief  Wrapper function that converts double to Eigen::VectorXd.
   */
  math::VectorXd get_values(const math::VectorXd& x) const final;

public:
  // Documentation inherited.
  using ConstraintSet::get_values;

  /**
   * @brief  Returns infinite bounds (e.g. no bounds).
   */
  optimization::VecBound get_bounds() const final;

  /**
   * Cost term printout slightly different from variables/constraints.
   */
  void show_info(double tol, int& index) const final;
};

} // namespace crobot::ik

