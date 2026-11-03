#pragma once

#include <memory>

#include "crobot/collision/collision_body.hpp"
#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class PairCollisionConstraint : public ik::Constraint {
public:
  using Ptr = std::shared_ptr<PairCollisionConstraint>;

  /**
   * @brief Create constraint to avoid collision between two geometries.
   *
   * @param name Name of this IK constraint.
   * @param rbtree The robot.
   * @param body1_name Name of the first body.
   * @param geom1_name Name of the geometry on the first body.
   * @param body2_name Name of the second body.
   * @param geom2_name Name of the geometry on the second body.
   * @param minimum_distance The minimum distance to maintain between
   * geometries.
   * @param detection_distance The distance at which collision avoidance
   * activates.
   */
  PairCollisionConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1_name,
      const std::string& geom1_name,
      const std::string& body2_name,
      const std::string& geom2_name,
      double minimum_distance,
      double detection_distance);

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
  kdl::ConstRigidBodyPtr m_body1;
  kdl::ConstRigidBodyPtr m_body2;
  std::string m_geom1_name;
  std::string m_geom2_name;
  double m_minimum_distance;
  double m_detection_distance;
  std::vector<int> m_qpos_addr;
};

} // namespace crobot::ik
