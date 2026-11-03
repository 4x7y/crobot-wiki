#include "crobot/ik/cons_bounding_box.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/bounds.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
BoundingBoxConstraint::BoundingBoxConstraint(
    const std::string& name,
    const kdl::ConstRigidBodyTreePtr& rbtree,
    const std::string& body_ee,
    const math::Vector3d& bpos_ee,
    const std::string& body_bbox,
    const math::Matrix4d& tform_bbox,
    const math::Vector3d& size_bbox,
    double tolerance)
  : Constraint(3, name, {"qpos"}),
    m_rbtree(rbtree),
    m_bpos_ee(bpos_ee),
    m_tform_bbox(tform_bbox),
    m_size_bbox(size_bbox),
    m_tolerance(tolerance) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  CROBOT_ASSERT_ERROR(tolerance >= 0.0);
  CROBOT_ASSERT_ERROR(size_bbox(0) >= 0.0);
  CROBOT_ASSERT_ERROR(size_bbox(1) >= 0.0);
  CROBOT_ASSERT_ERROR(size_bbox(2) >= 0.0);
  m_body_ee  = m_rbtree->get_body(body_ee);
  m_body_tar = m_rbtree->get_body(body_bbox);

  std::vector<int> qpos_addr_1 = m_body_ee->get_support_qpos_addr();
  std::vector<int> qpos_addr_2 = m_body_tar->get_support_qpos_addr();
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_1.begin(), qpos_addr_1.end());
  m_qpos_addr.insert(m_qpos_addr.end(), qpos_addr_2.begin(), qpos_addr_2.end());
  std::sort(m_qpos_addr.begin(), m_qpos_addr.end());
  m_qpos_addr.erase(
      std::unique(m_qpos_addr.begin(), m_qpos_addr.end()), m_qpos_addr.end());
}

//==============================================================================
math::VectorXd BoundingBoxConstraint::get_values(
    const math::VectorXd& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix4d xform_ee  = m_body_ee->get_xform(tforms);
  math::Matrix4d xform_ref = m_body_tar->get_xform(tforms);
  math::Matrix4d tform_ee2bbox
      = m_tform_bbox.tform_inv() * xform_ref.tform_inv() * xform_ee;
  math::Vector3d bpos_ee_at_bbox
      = (tform_ee2bbox * m_bpos_ee.cart2homo()).homo2cart_truncate();

  return bpos_ee_at_bbox;
}

//==============================================================================
VecBound BoundingBoxConstraint::get_bounds() const {
  VecBound b(get_rows());
  b.at(0) = Bounds(-m_size_bbox(0), m_size_bbox(0));
  b.at(1) = Bounds(-m_size_bbox(1), m_size_bbox(1));
  b.at(2) = Bounds(-m_size_bbox(2), m_size_bbox(2));
  return b;
}

//==============================================================================
void BoundingBoxConstraint::fill_jacobian_block(
    Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const {
  if (varsets->get_name() == "qpos") {
    // retrieve joint configurations
    math::VectorXd qpos = varsets->get_values_all();
    // forward kinematics
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

    math::Jacobian jacb_ee = m_body_ee->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::Jacobian jacb_tar = m_body_tar->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::LinearJacobian jacp_p
        = m_body_ee->get_jacp_at_bpos(tforms, jacb_ee, m_bpos_ee);
    math::LinearJacobian jacp_tar  = m_body_tar->get_jacp(jacb_tar);
    math::AngularJacobian jacr_tar = m_body_tar->get_jacr(jacb_tar);
    math::Vector3d xpos_p          = m_body_ee->get_xpos(tforms, m_bpos_ee);
    math::Vector3d xpos_tar        = m_body_tar->get_xpos(tforms);
    math::Matrix3d xmat_tar        = m_body_tar->get_xmat(tforms);

    math::Matrix3X cjacb
        = m_tform_bbox.tform2rotm().transpose()
          * ((xmat_tar.transpose() * (xpos_p - xpos_tar)).skew() * jacr_tar
             + xmat_tar.transpose() * (jacp_p - jacp_tar));

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j : m_qpos_addr) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
}
