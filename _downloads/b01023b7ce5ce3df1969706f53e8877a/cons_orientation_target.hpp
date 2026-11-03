#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class OrientationTargetConstraint : public ik::Constraint {
public:
  OrientationTargetConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name,
      const math::Quaternion& rotm_target,
      double tolerance = 0.01);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;
  std::vector<int> m_qpos_addr;
  math::Quaternion m_xquat_target;
  double m_tolerance;
};

} // namespace crobot::ik

