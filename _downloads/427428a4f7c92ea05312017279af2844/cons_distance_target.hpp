#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class DistanceConstraint : public ik::Constraint {
public:
  /**
   * @brief Create constraint on the distance between a body and a target.
   *
   * @param name Name of this IK constraint.
   * @param rbtree The robot.
   * @param body Name of the end-effector.
   * @param bpos Position of the body point in the body frame, specified as a
   * [x y z] vector.
   * @param xpos_target Position of the target in the world frame, specified as
   * a [x y z] vector.
   * @param tolerance Maximum allowed distance from target in three dimensions.
   */
  DistanceConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1,
      const math::Vector3d& bpos1,
      const std::string& body2,
      const math::Vector3d& bpos2,
      double distance,
      double tolerance);

  math::VectorXd get_values(const math::VectorXd& x) const override;

  optimization::VecBound get_bounds() const override;

  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body1;
  kdl::ConstRigidBodyPtr m_body2;
  std::vector<int> m_qpos_addr;
  math::Vector3d m_bpos1;
  math::Vector3d m_bpos2;
  double m_distance;
  double m_tolerance;
};

} // namespace crobot::ik

