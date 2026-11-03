#pragma once

#include "crobot/optimization/constraint_set.hpp"

namespace crobot::optimization {

class FrictionConeConstraint : public ConstraintSet {
public:
  FrictionConeConstraint(
      const std::string& conname,
      const std::string& varname_force,
      double mu,
      const math::Vector3d& cone_axis);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  void fill_jacobian_block(
      Component::Ptr varset, math::SparseJacobian& jac_block) const override;

  VecBound get_bounds() const override;

  void set_axis(const math::Vector3d& cone_axis);

private:
  double m_mu;
  math::Vector3d m_xvec;
  const std::string m_varname_force;
};

} // namespace crobot::optimization

