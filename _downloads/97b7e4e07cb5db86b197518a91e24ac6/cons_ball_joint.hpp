#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class BallJointConstraint : public ik::Constraint {
public:
  BallJointConstraint(
      const std::string& name, kdl::ConstRigidBodyPtr body, double tolerance);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyPtr m_body;
  std::pair<int, int> m_qpos_addr;
  double m_qpos_target;
  double m_tolerance;
};

} // namespace crobot::ik

