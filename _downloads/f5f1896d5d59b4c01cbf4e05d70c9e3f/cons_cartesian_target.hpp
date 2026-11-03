#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class CartesianConstraint : public ik::Constraint {
public:
  /**
   * @brief Create constraint on relative position of body
   *
   * @param name Name of this IK constraint.
   * @param rbtree The robot.
   * @param data MJData pointer for forward kinematics and Jacobian evaluation.
   * @param body Name of the end-effector.
   * @param bpos Position of the body point in the body frame, specified as a
   * [x y z] vector.
   * @param xpos_target Position of the target in the world frame, specified as
   * a [x y z] vector.
   * @param tolerance Maximum allowed distance from target in three dimensions.
   */
  CartesianConstraint(
      const std::string& name,
      const kdl::ConstRigidBodyTreePtr& rbtree,
      const std::string& body,
      const math::Vector3d& bpos,
      const math::Vector3d& lb,
      const math::Vector3d& ub);

  // Documentation inherited
  math::VectorXd get_values(const math::VectorXd& x) const override;

  // Documentation inherited
  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;
  std::vector<int> m_qpos_addr;
  math::Vector3d m_bpos;
  math::Vector3d m_lb;
  math::Vector3d m_ub;
  double m_tolerance;
};

} // namespace crobot::ik

