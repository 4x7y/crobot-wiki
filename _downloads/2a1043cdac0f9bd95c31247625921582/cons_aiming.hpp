#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

/**
 * @brief Aiming constraint for pointing at a target location.
 *
 * The AimingConstraint object describes a constraint that requires the z-axis
 * of one body (the end effector) to aim at a target point on another body (the
 * reference body). This constraint is satisfied if the aiming angle, a, between
 * the z-axis of the end-effector frame and the line connecting the end-effector
 * origin to the target point satisfies
 *
 *                       abs(a) <= AngularTolerance
 *
 * The position of the target point is defined relative to the reference body.
 */
class AimingConstraint : public ik::Constraint {
public:
  AimingConstraint(
      const std::string& name,
      const kdl::ConstRigidBodyTreePtr& rbtree,
      const std::string& body_name,
      const math::Vector3d& bpos,
      const math::Vector3d& bvec,
      const math::Vector3d& xpos_target,
      double tolerance = 0.0);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varset,
      math::SparseJacobian& jac_block) const override;

  void set_angular_tolerance(double tol);

  double get_angular_tolerance();

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;
  std::vector<int> m_qpos_addr;
  math::Vector3d m_bpos;
  math::Vector3d m_bvec;
  math::Vector3d m_xpos_target;
  double m_cos_angular_tol;
};

} // namespace crobot::ik

