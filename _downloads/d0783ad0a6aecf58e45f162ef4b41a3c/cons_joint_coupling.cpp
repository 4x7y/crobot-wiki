#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_joint_coupling.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
JointCouplingConstraint::JointCouplingConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& joint1_name,
    double coeff1,
    const std::string& joint2_name,
    double coeff2,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  CROBOT_ASSERT_ERROR(0.0 <= m_tolerance);
  kdl::ConstRigidBodyPtr body1 = m_rbtree->get_body_by_joint(joint1_name);
  CROBOT_ASSERT_ERROR(body1 != nullptr);
  std::pair<int, int> qpos_addr_1 = body1->get_qpos_addr();
  CROBOT_ASSERT_ERROR(
      qpos_addr_1.second == 1,
      "JointCouplingConstraint: body1 has more than one joint or its joint has "
      "more than 1 DOF. This constraint can only handle body with single 1DOF "
      "joint.");
  kdl::ConstRigidBodyPtr body2 = m_rbtree->get_body_by_joint(joint2_name);
  CROBOT_ASSERT_ERROR(body2 != nullptr);
  std::pair<int, int> qpos_addr_2 = body2->get_qpos_addr();
  CROBOT_ASSERT_ERROR(
      qpos_addr_2.second == 1,
      "JointCouplingConstraint: body2 has more than one joint or its joint has "
      "more than 1 DOF. This constraint can only handle body with single 1DOF "
      "joint.");
  m_qpos_addrs = std::make_pair<>(qpos_addr_1.first, qpos_addr_2.first);
  m_coeffs     = std::make_pair<>(coeff1, coeff2);
}

//==============================================================================
math::VectorXd JointCouplingConstraint::get_values(
    const math::VectorXd& qpos) const {
  return math::Vector1d(
      m_coeffs.first * qpos[m_qpos_addrs.first]
      + m_coeffs.second * qpos[m_qpos_addrs.second]);
};

//==============================================================================
VecBound JointCouplingConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_tolerance, m_tolerance);
  return b;
}

//==============================================================================
void JointCouplingConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    jac_block.coeffRef(0, m_qpos_addrs.first)  = m_coeffs.first;
    jac_block.coeffRef(0, m_qpos_addrs.second) = m_coeffs.second;
  }
}
