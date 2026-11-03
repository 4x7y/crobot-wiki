#include "crobot/ik/cons_obj_contact.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
ObjectContactConstraint::ObjectContactConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_obj,
    const math::Vector3d& bpos_obj,
    const math::Vector3d& bvec_obj,
    const std::string& body_rob,
    std::shared_ptr<const math::SurfacePatch> patch_rob,
    const std::string& varname_patch_rob,
    double tolerance_p,
    double tolerance_n)
  : ik::Constraint(6, name, {varname_patch_rob, "qpos"}),
    m_rbtree(rbtree),
    m_body_obj(m_rbtree->get_body(body_obj)),
    m_bpos_obj(bpos_obj),
    m_bvec_obj(bvec_obj),
    m_body_rob(m_rbtree->get_body(body_rob)),
    m_patch_rob(patch_rob),
    m_varname_patch_rob(varname_patch_rob) {
  m_bounds.resize(6);
  for (size_t i = 0; i < 3; i++) {
    m_bounds.at(i)     = optimization::Bounds(-tolerance_p, tolerance_p);
    m_bounds.at(i + 3) = optimization::Bounds(-tolerance_n, tolerance_n);
  }

  m_pose_addr = m_body_obj->get_support_qpos_addr();
}

//==============================================================================
math::VectorXd ObjectContactConstraint::get_values(
    const math::VectorXd& x) const {
  math::Vector2d uv_rob;
  math::VectorXd qpos;

  std::tie(uv_rob, qpos)  = parse_dependent_vars(x);
  math::VectorXd obj_pose = qpos.segment(m_pose_addr[0], 7);

  // position of the contact point should be the same in the world frame

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d bpos1 = m_patch_rob->evaluate(uv_rob);
  math::Vector3d bvec1 = m_patch_rob->normal(uv_rob);
  math::Vector3d xpos1 = m_body_rob->get_xpos(tforms, bpos1);
  math::Vector3d xvec1 = m_body_rob->get_xmat(tforms) * bvec1;

  math::Vector3d xpos2 = m_body_obj->get_xpos(tforms, m_bpos_obj);
  math::Vector3d xvec2 = m_body_obj->get_xmat(tforms) * m_bvec_obj;

  math::VectorXd ret(get_rows());
  ret.segment(0, 3) = xpos1 - xpos2;
  ret.segment(3, 3) = xvec1 + xvec2;

  return ret;
}

//==============================================================================
optimization::VecBound ObjectContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void ObjectContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {

  math::Vector2d uv_rob;
  math::VectorXd qpos;

  std::tie(uv_rob, qpos)
      = parse_dependent_vars(get_dependent_variable_values());
  math::VectorXd obj_pose = qpos.segment(m_pose_addr[0], 7);

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix3d xmat_obj = m_body_obj->get_xmat(tforms);
  math::Matrix3d xmat_rob = m_body_rob->get_xmat(tforms);

  math::Matrix6X cjacb;
  if (varset->get_name() == "qpos") {
    cjacb = math::Matrix6X::Zero(6, m_rbtree->get_qpos_dim());

    math::Vector3d bpos_rob = m_patch_rob->evaluate(uv_rob);
    math::Vector3d nvec_rob = m_patch_rob->normal(uv_rob);

    math::Jacobian jacb_rob = m_body_rob->get_jacb(
        qpos, tforms, kdl::JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);
    math::LinearJacobian jacp_rob
        = m_body_rob->get_jacp_at_bpos(tforms, jacb_rob, bpos_rob);
    math::AngularJacobian jacr_rob = m_body_rob->get_jacr(jacb_rob);
    math::LinearJacobian jacn_rob
        = m_body_rob->get_bvec_jacb(tforms, jacr_rob, nvec_rob);

    // first 3 rows for position
    math::Vector3d bpos_obj = m_bpos_obj;
    math::Vector3d xpos_obj = xmat_obj * bpos_obj;

    // partial (c,n) / partial joint angle
    math::Matrix6X J_rob = math::Matrix6X::Zero(6, m_rbtree->get_qpos_dim());
    J_rob.topRows(3)     = jacp_rob;
    J_rob.bottomRows(3)  = jacn_rob;

    // partial c / partial position
    math::Matrix3X dcdp = -math::Matrix3X::Identity(3, 3);
    // partial c / partial orientation
    math::Matrix34 J_r
        = math::Quaternion(obj_pose[3], obj_pose[4], obj_pose[5], obj_pose[6])
              .diff_omega_coeffs();
    math::Matrix3X dcdr = xpos_obj.skew() * J_r;

    // partial n / partial position
    math::Matrix3X dndp = math::Matrix3X::Zero(3, 3);
    // partial n / partial orientation
    math::Vector3d xvec_obj = xmat_obj * m_bvec_obj;
    math::Matrix3X dndr     = -xvec_obj.skew() * J_r;

    cjacb.leftCols(m_rbtree->get_qpos_dim()) = J_rob;
    cjacb.block<3, 3>(0, m_pose_addr[0])     = dcdp;
    cjacb.block<3, 4>(0, m_pose_addr[0] + 3) = dcdr;
    cjacb.block<3, 3>(3, m_pose_addr[0])     = dndp;

    cjacb.block<3, 4>(3, m_pose_addr[0] + 3) = dndr;

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_patch_rob) {
    cjacb = math::Matrix6X(6, 2);

    math::Matrix32 norm_jacb = m_patch_rob->normal_jacb(uv_rob);
    math::Matrix32 pos_jacb  = m_patch_rob->jacb(uv_rob);

    // top 3 rows for position
    cjacb.block<3, 2>(0, 0) = xmat_rob * pos_jacb;
    // bottom 3 rows for opposite vec
    cjacb.block<3, 2>(3, 0) = xmat_rob * norm_jacb;

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
}

//==============================================================================
std::tuple<math::Vector2d, math::VectorXd>
ObjectContactConstraint::parse_dependent_vars(const math::VectorXd& x) const {

  CROBOT_ASSERT_DEBUG(
      x.size() == 2 + m_rbtree->get_qpos_dim(), "Variable dimension mismatch");

  // [u_rob, v_rob], [qpos]
  math::Vector2d uv_rob = x.segment(0, 2);
  math::VectorXd qpos   = x.segment(2, m_rbtree->get_qpos_dim());
  return std::make_tuple(uv_rob, qpos);
}
