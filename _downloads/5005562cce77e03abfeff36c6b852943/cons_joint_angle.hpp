#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class JointAngleConstraint : public ik::Constraint {
public:
  JointAngleConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& joint_name,
      double qpos_target,
      double tolerance);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  int m_qpos_addr;
  double m_qpos_target;
  double m_tolerance;
};

} // namespace crobot::ik

