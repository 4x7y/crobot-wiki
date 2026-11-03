#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_orientation_target.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
OrientationTargetConstraint::OrientationTargetConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name,
    const math::Quaternion& quat_target,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_xquat_target(quat_target),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  m_body      = m_rbtree->get_body(body_name);
  m_qpos_addr = m_body->get_support_qpos_addr();
}

//==============================================================================
math::VectorXd OrientationTargetConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Quaternion xquat             = m_body->get_xquat(tforms);
  math::Quaternion quatdiff          = m_xquat_target * xquat.conjugate();
  math::AngleAxisd axang             = quatdiff.as_axang();
  math::Vector3d angerr              = axang.axis() * axang.angle();
  return math::Vector1d(std::sqrt(angerr.squaredNorm() + 1e-12));
};

//==============================================================================
VecBound OrientationTargetConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_tolerance, m_tolerance);
  return b;
}

//==============================================================================
void OrientationTargetConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos                = varset->get_values_all();
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
    math::Jacobian jacb                = m_body->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::AngularJacobian jacr = m_body->get_jacr(jacb);

    math::Quaternion xquat    = m_body->get_xquat(tforms);
    math::Quaternion quatdiff = m_xquat_target * xquat.conjugate();
    math::AngleAxisd axang    = quatdiff.as_axang();
    math::Vector3d angerr     = axang.axis() * axang.angle();
    double g                  = std::sqrt(angerr.squaredNorm() + 1e-12);

    math::Matrix1X jac = -angerr.transpose() * jacr / g;
    // set jacobian values
    for (int j : m_qpos_addr) {
      jac_block.coeffRef(0, j) = jac(j);
    }
  }
}
