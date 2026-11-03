#pragma once

#include "crobot/ik/variable.hpp"
#include "crobot/optimization/composite.hpp"
#include "crobot/optimization/constraint_set.hpp"

namespace crobot::ik {

class Constraint : public optimization::ConstraintSet {
public:
  Constraint(
      int n_constraints,
      const std::string& name,
      const std::vector<std::string>& variables = {});

  // Documentation inherited
  math::VectorXd get_dependent_variable_values() const override final;

  // Documentation inherited
  math::SparseJacobian get_jacobian() const override final;

  // Documentation inherited
  math::SparseJacobian get_jacobian_finite_diff() const override final;

protected:
  virtual void fill_jacobian_block(
      Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const = 0;

private:
};

} // namespace crobot::ik

