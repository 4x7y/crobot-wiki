#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_joint_angle.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
JointAngleConstraint::JointAngleConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& joint_name,
    double qpos_target,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  CROBOT_ASSERT_ERROR(0.0 <= m_tolerance);
  std::pair<int, int> qpos_addr
      = m_rbtree->get_body_by_joint(joint_name)->get_qpos_addr();
  CROBOT_ASSERT_ERROR(
      qpos_addr.second == 1,
      "JointAngleConstraint: body has more than one joint or its joint has "
      "more than 1 DOF. This constraint can only handle body with single 1DOF "
      "joint.");
  m_qpos_addr   = qpos_addr.first;
  m_qpos_target = qpos_target;
}

//==============================================================================
math::VectorXd JointAngleConstraint::get_values(
    const math::VectorXd& qpos) const {
  return math::Vector1d(qpos[m_qpos_addr] - m_qpos_target);
};

//==============================================================================
VecBound JointAngleConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_tolerance, m_tolerance);
  return b;
}

//==============================================================================
void JointAngleConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    jac_block.coeffRef(0, m_qpos_addr) = 1.0;
  }
}
