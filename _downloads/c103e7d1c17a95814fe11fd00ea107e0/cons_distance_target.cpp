#include "crobot/ik/cons_distance_target.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
DistanceConstraint::DistanceConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body1,
    const math::Vector3d& bpos1,
    const std::string& body2,
    const math::Vector3d& bpos2,
    double distance,
    double tolerance)
  : Constraint(1, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_bpos1(bpos1),
    m_bpos2(bpos2),
    m_distance(distance),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree != nullptr);
  m_body1 = m_rbtree->get_body(body1);
  m_body2 = m_rbtree->get_body(body2);

  std::vector<int> qpos_addr_1 = m_body1->get_support_qpos_addr();
  std::vector<int> qpos_addr_2 = m_body2->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_2.begin(), qpos_addr_2.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());
  CROBOT_ASSERT_ERROR(m_distance >= 0.0);
  CROBOT_ASSERT_ERROR(m_tolerance >= 0.0);
}

//==============================================================================
math::VectorXd DistanceConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Vector3d xpos1               = m_body1->get_xpos(tforms, m_bpos1);
  math::Vector3d xpos2               = m_body2->get_xpos(tforms, m_bpos2);
  return math::Vector1d((xpos1 - xpos2).norm() - m_distance);
};

//==============================================================================
VecBound DistanceConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_tolerance, m_tolerance);
  return b;
}

//==============================================================================
void DistanceConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() == "qpos") {
    math::VectorXd qpos                = varset->get_values_all();
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

    math::Jacobian jacb1 = m_body1->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::Jacobian jacb2 = m_body2->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);

    math::LinearJacobian jacp1
        = m_body1->get_jacp_at_bpos(tforms, jacb1, m_bpos1);
    math::LinearJacobian jacp2
        = m_body2->get_jacp_at_bpos(tforms, jacb2, m_bpos2);
    math::Vector3d xpos1 = m_body1->get_xpos(tforms, m_bpos1);
    math::Vector3d xpos2 = m_body2->get_xpos(tforms, m_bpos2);

    math::Vector3d xpos_err = xpos1 - xpos2;
    double g                = xpos_err.norm();
    math::Matrix1X J        = xpos_err.transpose() * (jacp1 - jacp2) / g;

    // do not use jacp.sparseView() here since sparsity pattern may change
    // during the computation
    for (int j : m_qpos_addr) {
      jac_block.coeffRef(0, j) = J(0, j);
    }
  }
}
