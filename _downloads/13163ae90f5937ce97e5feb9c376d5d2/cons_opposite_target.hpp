#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class OppositeOrientationConstraint : public ik::Constraint {
public:
  /**
   * @brief Create constraint on the distance between a body and a target.
   *
   * @param name Name of this IK constraint.
   * @param rbtree The robot.
   * @param body1 Name of the first body.
   * @param bvec1 Orientation vector in the body frame1.
   * @param body2 Name of the second body.
   * @param bvec2 Orientation vector in the body frame2.
   * @param tolerance Maximum allowed distance from target in three dimensions.
   */
  OppositeOrientationConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1,
      const math::Vector3d& bpos1,
      const math::Vector3d& bvec1,
      const std::string& body2,
      const math::Vector3d& bpos2,
      const math::Vector3d& bvec2,
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
  math::Vector3d m_bpos1;
  math::Vector3d m_bpos2;
  math::Vector3d m_bvec1;
  math::Vector3d m_bvec2;
  std::vector<int> m_qpos_addr;
  double m_tolerance;
};

} // namespace crobot::ik

