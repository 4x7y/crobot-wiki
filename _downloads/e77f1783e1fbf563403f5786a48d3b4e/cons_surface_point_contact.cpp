#include "crobot/ik/cons_surface_point_contact.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
SurfacePointContactConstraint::SurfacePointContactConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body1,
    std::shared_ptr<const math::SurfacePatch> patch1,
    const std::string& varname_patch1,
    const math::Vector3d& xpos_body2,
    const math::Vector3d& xvec_body2,
    double tolerance)
  : ik::Constraint(6, name, {varname_patch1, "qpos"}),
    m_rbtree(rbtree),
    m_body1(m_rbtree->get_body(body1)),
    m_patch1(patch1),
    m_varname_patch1(varname_patch1),
    m_xpos_body2(xpos_body2),
    m_xvec_body2(xvec_body2) {
  m_bounds.resize(6);
  for (size_t i = 0; i < 6; i++) {
    m_bounds.at(i) = optimization::Bounds(-tolerance, tolerance);
  }
  std::vector<int> qpos_addr_1 = m_body1->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());
}

//==============================================================================
math::VectorXd SurfacePointContactConstraint::get_values(
    const math::VectorXd& x) const {
  math::Vector2d uv1;
  math::VectorXd qpos;
  std::tie(uv1, qpos) = parse_dependent_vars(x);

  math::Vector3d bpos1 = m_patch1->evaluate(uv1);
  math::Vector3d bvec1 = m_patch1->normal(uv1);

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d xpos1 = m_body1->get_xpos(tforms, bpos1);
  math::Vector3d xpos2 = m_xpos_body2;
  math::Vector3d xvec1 = m_body1->get_xmat(tforms) * bvec1;
  math::Vector3d xvec2 = m_xvec_body2;

  math::VectorXd ret(get_rows());
  ret.segment(0, 3) = xpos1 - xpos2;
  ret.segment(3, 3) = xvec1 + xvec2;
  return ret;
}

//==============================================================================
optimization::VecBound SurfacePointContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void SurfacePointContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  math::Vector2d uv1;
  math::VectorXd qpos;
  std::tie(uv1, qpos) = parse_dependent_vars(get_dependent_variable_values());
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix6X cjacb;
  if (varset->get_name() == "qpos") {
    math::Vector3d bpos1 = m_patch1->evaluate(uv1);
    math::Vector3d nvec1 = m_patch1->normal(uv1);

    math::Jacobian jacb1 = m_body1->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::LinearJacobian jacp1
        = m_body1->get_jacp_at_bpos(tforms, jacb1, bpos1);
    math::AngularJacobian jacr1 = m_body1->get_jacr(jacb1);
    math::LinearJacobian jacn1  = m_body1->get_bvec_jacb(tforms, jacr1, nvec1);

    // math::Jacobian jacb2 = m_body2->get_jacb(
    //     qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    // math::LinearJacobian jacp2
    //     = m_body2->get_jacp_at_bpos(tforms, jacb2, bpos2);
    // math::AngularJacobian jacr2 = m_body2->get_jacr(jacb2);
    // math::LinearJacobian jacn2  = m_body2->get_bvec_jacb(tforms, jacr2,
    // nvec2);

    cjacb               = math::Matrix6X(6, m_rbtree->get_qpos_dim());
    cjacb.topRows(3)    = jacp1;
    cjacb.bottomRows(3) = jacn1;

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j : m_qpos_addr) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_patch1) {
    math::Matrix3d xmat1 = m_body1->get_xmat(tforms);
    cjacb                = math::Matrix6X(6, 2);
    cjacb.topRows(3)     = xmat1 * m_patch1->jacb(uv1);
    cjacb.bottomRows(3)  = xmat1 * m_patch1->normal_jacb(uv1);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
}

//==============================================================================
std::tuple<math::Vector2d, math::VectorXd>
SurfacePointContactConstraint::parse_dependent_vars(
    const math::VectorXd& x) const {
  CROBOT_ASSERT_DEBUG(
      x.size() == 2 + m_rbtree->get_qpos_dim(), "Variable dimension mismatch");

  // [u1, v1], [u2, v2], [qpos]
  math::Vector2d uv1  = x.segment(0, 2);
  math::VectorXd qpos = x.segment(2, m_rbtree->get_qpos_dim());
  return std::make_tuple(uv1, qpos);
}
