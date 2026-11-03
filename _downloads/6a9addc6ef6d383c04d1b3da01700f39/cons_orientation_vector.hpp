#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class OrientationVectorConstraint : public ik::Constraint {
public:
  OrientationVectorConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name,
      const math::Vector3d& bvec_body,
      const math::Vector3d& xvec_target,
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
  math::Vector3d m_bvec_body;
  math::Vector3d m_xvec_target;
  double m_tolerance;
  double m_cos_angular_tol;
};

} // namespace crobot::ik

