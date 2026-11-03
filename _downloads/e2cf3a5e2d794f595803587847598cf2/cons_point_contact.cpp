#include "crobot/ik/cons_point_contact.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
PointContactConstraint::PointContactConstraint(
    const std::string& name,
    const math::Vector3d& bpos,
    const math::Vector3d& xpos,
    double tolerance)
  : ik::Constraint(3, name, {"pose"}), m_bpos(bpos), m_xpos_target(xpos) {
  m_bounds.resize(3);
  for (size_t i = 0; i < 3; i++) {
    m_bounds.at(i) = optimization::Bounds(-tolerance, tolerance);
  }
}

//==============================================================================
math::VectorXd PointContactConstraint::get_values(
    const math::VectorXd& x) const {

  math::Vector3d body_xpos = x.segment(0, 3);
  math::Vector3d xpos      = m_bpos + body_xpos;

  return (xpos - m_xpos_target);
}

//==============================================================================
optimization::VecBound PointContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void PointContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "pose") {

    for (int i = 0; i < 3; i++) {
      jac_block.coeffRef(i, i) = 1;
    }
  }
}

//==============================================================================
void PointContactConstraint::set_xpos(const math::Vector3d& xpos) {
  m_xpos_target = xpos;
}
