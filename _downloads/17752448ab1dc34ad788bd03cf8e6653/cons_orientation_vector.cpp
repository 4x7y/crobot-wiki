#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_orientation_vector.hpp"
#include "crobot/math/common.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
OrientationVectorConstraint::OrientationVectorConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name,
    const math::Vector3d& bvec_body,
    const math::Vector3d& xvec_target,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  m_body      = m_rbtree->get_body(body_name);
  m_qpos_addr = m_body->get_support_qpos_addr();
  CROBOT_ASSERT_ERROR(0 <= tolerance && tolerance <= M_PI);
  m_cos_angular_tol = std::cos(tolerance);
  CROBOT_ASSERT_ERROR(bvec_body.norm() > math::constantsd::epsilon());
  m_bvec_body = bvec_body.normalized();
  CROBOT_ASSERT_ERROR(xvec_target.norm() > math::constantsd::epsilon());
  m_xvec_target = xvec_target.normalized();
}

//==============================================================================
math::VectorXd OrientationVectorConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Vector3d z                   = m_body->get_xmat(tforms) * m_bvec_body;
  double g = (m_xvec_target.transpose() * z).value() - m_cos_angular_tol;
  return math::Vector1d(g);
};

//==============================================================================
VecBound OrientationVectorConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = BoundGreaterZero;
  return b;
}

//==============================================================================
void OrientationVectorConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos = varset->get_values_all();

    // Jacobian
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
    math::Jacobian jacb                = m_body->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);

    // rdot = JTwistV*[wE, vE] where wE and vE are the angular and
    // linear rates of change of T.
    math::Matrix36 JTwistR;
    JTwistR << math::Matrix3d::Zero(), -math::Matrix3d::Identity();

    // z - Unit vector along z-axis of end-effector frame, expressed
    // in the world frame
    math::Vector3d z = m_body->get_xmat(tforms) * m_bvec_body;

    // zdot = JTwistZ*[vE, wE] where vE and wE are the linear and angular rates
    // of change of T.
    math::Matrix36 JTwistZ;
    JTwistZ << -z.skew(), math::Matrix3d::Zero();

    // The cosine of the angle, a,  between the z-axis of the end effector and
    // the line between the end-effector origin and target is given by:
    //
    //   cos(a) = (v'*z)/norm(v)
    //
    // we want to ensure that
    //
    //   cos(m_cos_angular_tol) <= cos(a)
    //
    // This is equivalent to
    //
    //   0 <= g
    //
    // where
    //
    //   g = v'*z - norm(v)*cos(m_cos_angular_tol)
    //
    math::RowVector3d dgdr
        = z.transpose() - m_cos_angular_tol * m_xvec_target.transpose();
    math::RowVector3d dgdz = m_xvec_target.transpose();

    // gdot = JTwist*[vE, wE] where vE and wE are the linear and angular rates
    // of change of T.
    math::RowVector6d JTwist = dgdr * JTwistR + dgdz * JTwistZ;
    math::RowVectorXd jacc   = JTwist * jacb;

    // set jacobian values
    for (int j : m_qpos_addr) {
      jac_block.coeffRef(0, j) = jacc(j);
    }
  }
}
