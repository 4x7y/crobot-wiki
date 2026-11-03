#pragma once

#include "crobot/optimization/constraint_set.hpp"

namespace crobot::optimization {

/**
 * @brief A container holding a single cost term.
 *
 * This container builds a scalar cost term from the values of the variables.
 * This can be seen as a constraint with only one row and no bounds.
 */
class CostTerm : public ConstraintSet {
public:
  CostTerm(const std::string& name, const std::vector<std::string>& variables);
  virtual ~CostTerm() = default;

  /**
   * @brief  Returns the scalar cost term calculated from the @c variables.
   */
  virtual double get_cost(const math::VectorXd& x) const = 0;

private:
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
  VecBound get_bounds() const final;

  /**
   * Cost term printout slightly different from variables/constraints.
   */
  void show_info(double tol, int& index) const final;
};

} // namespace crobot::optimization
