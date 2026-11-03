#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/distance.h>

#include "crobot/collision/geometry.hpp"
#include "crobot/common/assert.hpp"
#include "crobot/ik/cons_pair_collision.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
PairCollisionConstraint::PairCollisionConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body1_name,
    const std::string& geom1_name,
    const std::string& body2_name,
    const std::string& geom2_name,
    double minimum_distance,
    double detection_distance)
  : ik::Constraint(1, name, {"qpos"}),
    m_rbtree(rbtree),
    m_geom1_name(geom1_name),
    m_geom2_name(geom2_name),
    m_minimum_distance(minimum_distance),
    m_detection_distance(detection_distance) {
  CROBOT_ASSERT_ERROR(m_rbtree != nullptr);

  // Get bodies and cast to CollisionBody
  m_body1 = m_rbtree->get_body(body1_name);
  m_body2 = m_rbtree->get_body(body2_name);

  m_body1->get_geom(m_geom1_name);
  m_body2->get_geom(m_geom2_name);

  // Collect qpos addresses from both bodies
  std::vector<int> qpos_addr_1 = m_body1->get_support_qpos_addr();
  std::vector<int> qpos_addr_2 = m_body2->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_2.begin(), qpos_addr_2.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());

  CROBOT_ASSERT_ERROR(m_minimum_distance >= 0.0);
  CROBOT_ASSERT_ERROR(m_detection_distance >= m_minimum_distance);
}

//==============================================================================
math::VectorXd PairCollisionConstraint::get_values(
    const math::VectorXd& qpos) const {
  // Compute forward kinematics
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  // Get geometries and their local transforms
  auto [geom1, tform1] = m_body1->get_geom(m_geom1_name);
  auto [geom2, tform2] = m_body2->get_geom(m_geom2_name);

  // Compute world transforms for each geometry
  math::Matrix4d geom1_xform = m_body1->get_xform(tforms) * tform1;
  math::Matrix4d geom2_xform = m_body2->get_xform(tforms) * tform2;

  // Create collision objects
  hpp::fcl::CollisionObject obj1(
      geom1, collision::to_fcl_transform(geom1_xform));
  hpp::fcl::CollisionObject obj2(
      geom2, collision::to_fcl_transform(geom2_xform));

  // Perform distance query
  hpp::fcl::DistanceRequest request(true); // Enable nearest points
  hpp::fcl::DistanceResult result;
  hpp::fcl::distance(&obj1, &obj2, request, result);

  // If distance is greater than detection distance, constraint is inactive
  if (result.min_distance > m_detection_distance) {
    return math::Vector1d(-1e6);
  }

  // Return constraint value: (minimum_distance - actual_distance)
  // Constraint is satisfied when this value <= 0
  return math::Vector1d(m_minimum_distance - result.min_distance);
}

//==============================================================================
optimization::VecBound PairCollisionConstraint::get_bounds() const {
  optimization::VecBound bounds(get_rows());
  bounds.at(0)
      = optimization::Bounds(0.0, std::numeric_limits<double>::infinity());
  return bounds;
}

//==============================================================================
void PairCollisionConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    // For now, set jacobian to zero (trivial implementation)
    // TODO: Implement proper jacobian computation using nearest points
    for (int j : m_qpos_addr) {
      jac_block.coeffRef(0, j) = 0.0;
    }
  }
}
