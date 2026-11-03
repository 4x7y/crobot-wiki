#include "crobot/common/assert.hpp"
#include "crobot/ik/cons_surface_curve_contact.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
SurfaceCurveContactConstraint::SurfaceCurveContactConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body1,
    std::shared_ptr<const math::SurfacePatch> patch,
    const std::string& varname_patch,
    const std::string& body2,
    std::shared_ptr<const math::CurveSegment> segment,
    const std::string& varname_segment,
    double tolerance)
  : ik::Constraint(4, name, {varname_patch, varname_segment, "qpos"}),
    m_rbtree(rbtree),
    m_body1(m_rbtree->get_body(body1)),
    m_body2(m_rbtree->get_body(body2)),
    m_patch(patch),
    m_segment(segment),
    m_varname_patch(varname_patch),
    m_varname_segment(varname_segment) {
  m_bounds.resize(4);
  for (size_t i = 0; i < 4; i++) {
    m_bounds.at(i) = optimization::Bounds(-tolerance, tolerance);
  }
  std::vector<int> qpos_addr_1 = m_body1->get_support_qpos_addr();
  std::vector<int> qpos_addr_2 = m_body1->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_2.begin(), qpos_addr_2.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());
}

//==============================================================================
math::VectorXd SurfaceCurveContactConstraint::get_values(
    const math::VectorXd& x) const {
  math::Vector2d uv1;
  double u2;
  math::VectorXd qpos;
  std::tie(uv1, u2, qpos) = parse_dependent_vars(x);

  math::Vector3d bpos1 = m_patch->evaluate(uv1);
  math::Vector3d bpos2 = m_segment->evaluate(u2);
  math::Vector3d bvec1 = m_patch->normal(uv1);
  math::Vector3d bvec2 = m_segment->tangent(u2);

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d xpos1 = m_body1->get_xpos(tforms, bpos1);
  math::Vector3d xpos2 = m_body2->get_xpos(tforms, bpos2);
  math::Vector3d xvec1 = m_body1->get_xmat(tforms) * bvec1;
  math::Vector3d xvec2 = m_body2->get_xmat(tforms) * bvec2;

  math::VectorXd ret(get_rows());
  ret.segment(0, 3) = xpos1 - xpos2;
  ret(3)            = xvec1.dot(xvec2);
  return ret;
}

//==============================================================================
optimization::VecBound SurfaceCurveContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void SurfaceCurveContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {
  math::Vector2d uv1;
  double u2;
  math::VectorXd qpos;
  std::tie(uv1, u2, qpos)
      = parse_dependent_vars(get_dependent_variable_values());
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix4X cjacb;
  if (varset->get_name() == "qpos") {
    math::Vector3d bpos1 = m_patch->evaluate(uv1);
    math::Vector3d bpos2 = m_segment->evaluate(u2);
    math::Vector3d nvec1 = m_patch->normal(uv1);
    math::Vector3d tvec2 = m_segment->tangent(u2);
    math::Vector3d xvec1 = m_body1->get_xmat(tforms) * nvec1;
    math::Vector3d xvec2 = m_body2->get_xmat(tforms) * tvec2;

    math::Jacobian jacb1 = m_body1->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::LinearJacobian jacp1
        = m_body1->get_jacp_at_bpos(tforms, jacb1, bpos1);
    math::AngularJacobian jacr1 = m_body1->get_jacr(jacb1);
    math::LinearJacobian jacn1  = m_body1->get_bvec_jacb(tforms, jacr1, nvec1);

    math::Jacobian jacb2 = m_body2->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::LinearJacobian jacp2
        = m_body2->get_jacp_at_bpos(tforms, jacb2, bpos2);
    math::AngularJacobian jacr2 = m_body2->get_jacr(jacb2);
    math::LinearJacobian jact2  = m_body2->get_bvec_jacb(tforms, jacr2, tvec2);

    cjacb            = math::Matrix4X(4, m_rbtree->get_qpos_dim());
    cjacb.topRows(3) = jacp1 - jacp2;
    cjacb.row(3)     = xvec1.transpose() * jact2 + xvec2.transpose() * jacn1;

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j : m_qpos_addr) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_patch) {
    math::Matrix3d xmat1 = m_body1->get_xmat(tforms);
    math::Vector3d tvec2 = m_segment->tangent(u2);
    math::Vector3d xvec2 = m_body2->get_xmat(tforms) * tvec2;

    cjacb            = math::Matrix4X(4, 2);
    cjacb.topRows(3) = xmat1 * m_patch->jacb(uv1);
    cjacb.row(3)     = xvec2.transpose() * xmat1 * m_patch->normal_jacb(uv1);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_segment) {
    math::Matrix3d xmat2 = m_body2->get_xmat(tforms);
    math::Vector3d nvec1 = m_patch->normal(uv1);
    math::Vector3d xvec1 = m_body1->get_xmat(tforms) * nvec1;

    cjacb            = math::Matrix4X(4, 1);
    cjacb.topRows(3) = -xmat2 * m_segment->jacb(u2);
    cjacb.row(3)     = xvec1.transpose() * xmat2 * m_segment->tangent_jacb(u2);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
}

//==============================================================================
std::tuple<math::Vector2d, double, math::VectorXd>
SurfaceCurveContactConstraint::parse_dependent_vars(
    const math::VectorXd& x) const {
  CROBOT_ASSERT_DEBUG(
      x.size() == 2 + 1 + m_rbtree->get_qpos_dim(),
      "Variable dimension mismatch");

  // [u1, v1], [u2], [qpos]
  math::Vector2d uv1  = x.segment(0, 2);
  double u2           = x[2];
  math::VectorXd qpos = x.segment(3, m_rbtree->get_qpos_dim());
  return std::make_tuple(uv1, u2, qpos);
}
