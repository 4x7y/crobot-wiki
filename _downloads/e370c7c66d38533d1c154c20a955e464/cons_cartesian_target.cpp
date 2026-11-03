#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/ik/cons_cartesian_target.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
CartesianConstraint::CartesianConstraint(
    const std::string& name,
    const kdl::ConstRigidBodyTreePtr& rbtree,
    const std::string& body_name,
    const math::Vector3d& bpos,
    const math::Vector3d& lb,
    const math::Vector3d& ub)
  : Constraint(3, name, {"qpos"}),
    m_rbtree(rbtree),
    m_bpos(bpos),
    m_lb(lb),
    m_ub(ub) {
  CROBOT_ASSERT_ERROR(rbtree);
  m_body      = rbtree->get_body(body_name);
  m_qpos_addr = m_body->get_support_qpos_addr();
}

//==============================================================================
math::VectorXd CartesianConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d xpos;
  if (m_bpos.isZero()) {
    xpos = m_body->get_xpos(tforms);
  }
  else {
    xpos = m_body->get_xpos(tforms, m_bpos);
  }

  return xpos;
};

//==============================================================================
VecBound CartesianConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(m_lb(0), m_ub(0));
  b.at(1) = Bounds(m_lb(1), m_ub(1));
  b.at(2) = Bounds(m_lb(2), m_ub(2));
  return b;
}

//==============================================================================
void CartesianConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos                = varset->get_values_all();
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

    math::Jacobian jacb = m_body->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);

    math::LinearJacobian jacp;
    if (m_bpos.isZero()) {
      jacp = m_body->get_jacp(jacb);
    }
    else {
      jacp = m_body->get_jacp_at_bpos(tforms, jacb, m_bpos);
    }

    // do not use jacp.sparseView() here since sparsity pattern may change
    // during the computation
    for (size_t i = 0; i < 3; i++) {
      for (int j : m_qpos_addr) {
        jac_block.coeffRef(i, j) = jacp(i, j);
      }
    }
  }
}
