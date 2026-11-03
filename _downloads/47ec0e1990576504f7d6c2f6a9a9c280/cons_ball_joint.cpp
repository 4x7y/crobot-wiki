#include "crobot/ik/cons_ball_joint.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/kdl/rigid_body_joint.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
BallJointConstraint::BallJointConstraint(
    const std::string& name, kdl::ConstRigidBodyPtr body, double tolerance)
  : Constraint(1, name, {"qpos"}), m_body(body), m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_body);
  CROBOT_ASSERT_ERROR(m_body->get_joint()->get_type() == kdl::JointType::BALL);
  CROBOT_ASSERT_ERROR(tolerance >= 0.0);
  m_qpos_addr = body->get_joint()->get_qpos_addr()[0];
}

//==============================================================================
math::VectorXd BallJointConstraint::get_values(const math::VectorXd& x) const {
  return math::Vector1d(
      x.segment(m_qpos_addr.first, m_qpos_addr.second).squaredNorm() - 1.0);
}

//==============================================================================
VecBound BallJointConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_tolerance, m_tolerance);
  return b;
}

//==============================================================================
void BallJointConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos = varset->get_values_all();

    for (int i = 0; i < m_qpos_addr.second; i++) {
      int idx = m_qpos_addr.first + i;

      jac_block.coeffRef(0, idx) = 2.0 * qpos(idx);
    }
  }
}
