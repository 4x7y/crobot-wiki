#pragma once

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class BoundingBoxConstraint : public Constraint {
public:
  /**
   * @brief Construct a new Bounding Box Constraint object
   *
   * @param name Name of this constraint
   * @param rbtree The robot
   * @param body Name of the end-effector body
   * @param bpos Position of the end-effector point in the body frame
   * @param body_bbox Name of the body that the target bounding box is fixed on
   * @param bpos_bbox Position of the target bounding box center in the
   * reference body
   * @param rotm_bbox Rotation matrix of the target bounding box in the
   * reference body
   * @param size_bbox Half-sizes of the target bounding box in the reference
   * @param tolerance Maximum allowed distance tolerance
   */
  BoundingBoxConstraint(
      const std::string& name,
      const kdl::ConstRigidBodyTreePtr& rbtree,
      const std::string& body,
      const math::Vector3d& bpos,
      const std::string& body_bbox,
      const math::Matrix4d& tform_bbox,
      const math::Vector3d& size_bbox,
      double tolerance);

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
  kdl::ConstRigidBodyPtr m_body_ee;
  kdl::ConstRigidBodyPtr m_body_tar;

  std::vector<int> m_qpos_addr;

  math::Vector3d m_bpos_ee;
  math::Matrix4d m_tform_bbox;
  math::Vector3d m_size_bbox;

  double m_tolerance;
};

} // namespace crobot::ik

