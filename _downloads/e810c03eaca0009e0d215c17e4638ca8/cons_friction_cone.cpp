#include "crobot/optimization/cons_friction_cone.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
FrictionConeConstraint::FrictionConeConstraint(
    const std::string& conname,
    const std::string& varname_force,
    double mu,
    const math::Vector3d& cone_axis)
  : ConstraintSet(2, conname, {varname_force}),
    m_mu(mu),
    m_xvec(cone_axis.normalized()),
    m_varname_force(varname_force) {
}

//==============================================================================
math::VectorXd FrictionConeConstraint::get_values(
    const math::VectorXd& x) const {
  math::Vector3d xfrc = x;

  math::VectorXd value(2);
  value(0) = m_xvec.dot(xfrc.normalized()) - std::cos(std::atan(m_mu));
  value(1) = m_xvec.dot(xfrc);
  return value;
}

//==============================================================================
void FrictionConeConstraint::fill_jacobian_block(
    Component::Ptr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == m_varname_force) {
    math::Vector3d xfrc = varset->get_values();
    double fnorm2       = xfrc.squaredNorm();
    double fnorm        = std::sqrt(fnorm2);

    math::RowVector3d jacb = (m_xvec.transpose() * fnorm
                              - m_xvec.dot(xfrc) * xfrc.transpose() / fnorm)
                             / fnorm2;

    jac_block.coeffRef(0, 0) = jacb(0);
    jac_block.coeffRef(0, 1) = jacb(1);
    jac_block.coeffRef(0, 2) = jacb(2);
    jac_block.coeffRef(1, 0) = m_xvec(0);
    jac_block.coeffRef(1, 1) = m_xvec(1);
    jac_block.coeffRef(1, 2) = m_xvec(2);
  }
}

//==============================================================================
VecBound FrictionConeConstraint::get_bounds() const {
  VecBound bounds(2);
  bounds.at(0) = BoundGreaterZero;
  bounds.at(1) = BoundGreaterZero;
  return bounds;
}

//==============================================================================
void FrictionConeConstraint::set_axis(const math::Vector3d& cone_axis) {
  m_xvec = cone_axis.normalized();
}
