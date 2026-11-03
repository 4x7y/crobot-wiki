#include "crobot/ik/cons_opposite_target.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
OppositeOrientationConstraint::OppositeOrientationConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body1,
    const math::Vector3d& bpos1,
    const math::Vector3d& bvec1,
    const std::string& body2,
    const math::Vector3d& bpos2,
    const math::Vector3d& bvec2,
    double tolerance)
  : Constraint(2, name, {"qpos"}),
    m_rbtree(std::move(rbtree)),
    m_bpos1(bpos1),
    m_bpos2(bpos2),
    m_bvec1(bvec1),
    m_bvec2(bvec2),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree != nullptr);
  m_body1 = m_rbtree->get_body(body1);
  m_body2 = m_rbtree->get_body(body2);

  m_bvec1.normalize();
  m_bvec2.normalize();

  std::vector<int> qpos_addr_1 = m_body1->get_support_qpos_addr();
  std::vector<int> qpos_addr_2 = m_body2->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_2.begin(), qpos_addr_2.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());
  CROBOT_ASSERT_ERROR(m_tolerance >= 0.0);
}

//==============================================================================
math::VectorXd OppositeOrientationConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d xpos1 = m_body1->get_xpos(tforms, m_bpos1);
  math::Vector3d xpos2 = m_body2->get_xpos(tforms, m_bpos2);
  math::Vector3d xvec1 = m_body1->get_xmat(tforms) * m_bvec1;
  math::Vector3d xvec2 = m_body2->get_xmat(tforms) * m_bvec2;

  double f1 = xvec1.dot(xpos1 - xpos2); // + 1.0;
  double f2 = xvec2.dot(xpos1 - xpos2); // - 1.0;

  return math::Vector2d(f1, f2);
}

//==============================================================================
VecBound OppositeOrientationConstraint::get_bounds() const {
  VecBound b(get_rows());
  // b.at(0) = Bounds(-m_tolerance, m_tolerance);
  // b.at(1) = Bounds(-m_tolerance, m_tolerance);

  b.at(0) = Bounds(-1, -std::cos(m_tolerance));
  b.at(1) = Bounds(std::cos(m_tolerance), 1);
  return b;
}

//==============================================================================
void OppositeOrientationConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {

  // jac_block = get_jacobian_finite_diff();
  // return;

  if (varset->get_name() == "qpos") {
    math::VectorXd qpos                = varset->get_values_all();
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

    math::Jacobian jacb1 = m_body1->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::Jacobian jacb2 = m_body2->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);

    math::AngularJacobian jacr1 = m_body1->get_jacr(jacb1);
    math::AngularJacobian jacr2 = m_body2->get_jacr(jacb2);
    math::LinearJacobian jacp1
        = m_body1->get_jacp_at_bpos(tforms, jacb1, m_bpos1);
    math::LinearJacobian jacp2
        = m_body2->get_jacp_at_bpos(tforms, jacb2, m_bpos2);
    math::Matrix3d xmat1 = m_body1->get_xmat(tforms);
    math::Matrix3d xmat2 = m_body2->get_xmat(tforms);

    math::Vector3d xpos1 = m_body1->get_xpos(tforms, m_bpos1);
    math::Vector3d xpos2 = m_body2->get_xpos(tforms, m_bpos2);
    math::Vector3d xvec  = xpos1 - xpos2;

    math::Vector3d xvec1 = xmat1 * m_bvec1;
    math::Vector3d xvec2 = xmat2 * m_bvec2;

    math::Matrix3X g = jacp1 - jacp2;

    math::Matrix1X p1
        = -xvec.transpose() * ((xvec1).skew() * jacr1) + xvec1.transpose() * g;

    math::Matrix1X p2
        = -xvec.transpose() * ((xvec2).skew() * jacr2) + xvec2.transpose() * g;

    // do not use jacp.sparseView() here since sparsity pattern may change
    // during the computation
    for (int j : m_qpos_addr) {
      jac_block.coeffRef(0, j) = p1(0, j);
      jac_block.coeffRef(1, j) = p2(0, j);
    }
  }
}
