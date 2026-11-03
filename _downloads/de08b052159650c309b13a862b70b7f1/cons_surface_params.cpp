#include "crobot/ik/cons_surface_params.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
SurfaceParamsConstraint::SurfaceParamsConstraint(
    const std::string& name,
    const std::string& varname,
    const math::Vector2d& uv_est,
    double tolerance)
  : ik::Constraint(2, name, {varname}), m_varname(varname), m_uv_est(uv_est) {
  m_bounds.resize(2);
  for (size_t i = 0; i < 2; i++) {
    m_bounds.at(i) = optimization::Bounds(-tolerance, tolerance);
  }
}

//==============================================================================
math::VectorXd SurfaceParamsConstraint::get_values(
    const math::VectorXd& x) const {

  math::Vector2d diff = x - m_uv_est;

  return diff;
}

//==============================================================================
optimization::VecBound SurfaceParamsConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void SurfaceParamsConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == m_varname) {
    for (int i = 0; i < 2; i++) {
      jac_block.coeffRef(i, i) = 1;
    }
  }
}

//==============================================================================
void SurfaceParamsConstraint::set_uv_est(const math::Vector2d& uv) {
  m_uv_est = uv;
}
