#include "crobot/ik/cons_plane_contact.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
PlaneContactConstraint::PlaneContactConstraint(
    const std::string& name,
    const math::Vector3d& bpos,
    const math::Vector3d& point,
    const math::Vector3d& normal,
    double tolerance)
  : ik::Constraint(1, name, {"pose"}),
    m_bpos(bpos),
    m_point(point),
    m_normal(normal) {
  m_bounds.resize(1);
  m_bounds.at(0) = optimization::Bounds(-tolerance, tolerance);
}

//==============================================================================
math::VectorXd PlaneContactConstraint::get_values(
    const math::VectorXd& x) const {

  math::Vector3d body_xpos  = x.segment(0, 3);
  math::Vector4d body_xquat = x.segment(3, 4).normalized();

  math::Matrix3d xmat
      = math::Quaternion(
            body_xquat[0], body_xquat[1], body_xquat[2], body_xquat[3])
            .as_rotm();

  math::Vector3d xpos = xmat * m_bpos + body_xpos;

  return m_normal.transpose() * (xpos - m_point);
}

//==============================================================================
optimization::VecBound PlaneContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void PlaneContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "pose") {
    math::VectorXd pose = varset->get_values_all();
    pose.segment(3, 4)  = pose.segment(3, 4).normalized();

    math::Quaternion body_quat
        = math::Quaternion(pose[3], pose[4], pose[5], pose[6]);
    math::Matrix3d body_xmat = body_quat.as_rotm();

    math::Matrix34 J_r    = body_quat.diff_omega_coeffs();
    math::Matrix3X J_1    = math::Matrix3X::Zero(3, 7);
    J_1.block<3, 4>(0, 3) = J_r;
    math::Matrix3X J_2    = math::Matrix3X::Zero(3, 7);
    J_2.block<3, 3>(0, 0) = math::Matrix3X::Identity(3, 3);

    math::Matrix1X cjacb = math::Matrix1X::Zero(1, 7);
    cjacb = m_normal.transpose() * (-(body_xmat * m_bpos).skew() * J_1 + J_2);

    for (int i = 0; i < 7; i++) {
      jac_block.coeffRef(0, i) = cjacb(0, i);
    }
  }
}

//==============================================================================
void PlaneContactConstraint::set_point(const math::Vector3d& point) {
  m_point = point;
}

//==============================================================================
void PlaneContactConstraint::set_normal(const math::Vector3d& normal) {
  m_normal = normal;
}

//==============================================================================
void PlaneContactConstraint::set_plane(
    const math::Vector3d& point, const math::Vector3d& normal) {
  m_point  = point;
  m_normal = normal;
}
