#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_aiming.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
AimingConstraint::AimingConstraint(
    const std::string& name,
    const kdl::ConstRigidBodyTreePtr& rbtree,
    const std::string& body_name,
    const math::Vector3d& bpos,
    const math::Vector3d& bvec,
    const math::Vector3d& xpos_target,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(rbtree),
    m_bpos(bpos),
    m_bvec(bvec),
    m_xpos_target(xpos_target),
    m_cos_angular_tol(std::cos(tolerance)) {
  CROBOT_ASSERT_ERROR(rbtree);
  CROBOT_ASSERT_ERROR(0.0 <= m_cos_angular_tol && m_cos_angular_tol <= 1.0);
  CROBOT_ASSERT_ERROR(
      m_bvec.norm() >= math::constantsd::epsilon(),
      "Body reference vector can not be a zero vector.");
  if (!math::is_equal(m_bvec.norm(), 1.0)) {
    CROBOT_ASSERT_WARNING(
        "The bvec passed to the AimingConstraint is not a unit vector. "
        "Normalize the bvec to a unit vector.");
    m_bvec.normalize();
  }

  m_body      = rbtree->get_body(body_name);
  m_qpos_addr = m_body->get_support_qpos_addr();
}

//==============================================================================
math::VectorXd AimingConstraint::get_values(const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Vector3d xpos                = m_body->get_xpos(tforms, m_bpos);

  // r - Vector from the origin of the end-effector frame to the
  // target point, expressed in the world frame
  math::Vector3d r = m_xpos_target - xpos;

  // z - Unit vector along z-axis of end-effector frame, expressed
  // in the world frame
  math::Vector3d z;
  if (m_bvec.isZero()) {
    z = m_body->get_xmat(tforms).block<3, 1>(0, 2);
  }
  else {
    z = m_body->get_xmat(tforms) * m_bvec;
  }

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
  double r_norm = std::sqrt(r.squaredNorm() + 1e-18);
  double g      = (r.transpose() * z).value() - r_norm * m_cos_angular_tol;

  return math::Vector1d(g);
};

//==============================================================================
VecBound AimingConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = BoundGreaterZero;
  return b;
}

//==============================================================================
void AimingConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos                = varset->get_values_all();
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
    math::Jacobian jacb_body           = m_body->get_jacb(tforms);
    math::Jacobian jacb = m_body->get_jacb_at_bpos(tforms, jacb_body, m_bpos);
    math::Vector3d xpos = m_body->get_xpos(tforms, m_bpos);

    // r - Vector from the origin of the end-effector frame to the
    // target point, expressed in the world frame
    math::Vector3d r = m_xpos_target - xpos;

    // rdot = JTwistV*[wE, vE] where wE and vE are the angular and
    // linear rates of change of T.
    math::Matrix36 JTwistR;
    JTwistR << math::Matrix3d::Zero(), -math::Matrix3d::Identity();

    // z - Unit vector along z-axis of end-effector frame, expressed
    // in the world frame
    math::Vector3d z;
    if (m_bvec.isZero()) {
      z = m_body->get_xmat(tforms).block<3, 1>(0, 2);
    }
    else {
      z = m_body->get_xmat(tforms) * m_bvec;
    }

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
    double r_norm = std::sqrt(r.squaredNorm() + 1e-18);
    math::RowVector3d dgdr
        = z.transpose() - (m_cos_angular_tol / r_norm) * r.transpose();
    math::RowVector3d dgdz = r.transpose();

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

//==============================================================================
void AimingConstraint::set_angular_tolerance(double tol) {
  m_cos_angular_tol = std::cos(tol);
}

//==============================================================================
double AimingConstraint::get_angular_tolerance() {
  return std::acos(m_cos_angular_tol);
}
