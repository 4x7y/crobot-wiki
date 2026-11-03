#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class JointCouplingConstraint : public ik::Constraint {
public:
  JointCouplingConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& joint1_name,
      double coeff1,
      const std::string& joint2_name,
      double coeff2,
      double tolerance);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  std::pair<int, int> m_qpos_addrs;
  std::pair<double, double> m_coeffs;
  double m_tolerance;
};

} // namespace crobot::ik

