#include "crobot/ik/cons_body_contact.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
BodyContactConstraint::BodyContactConstraint(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_obj,
    std::shared_ptr<const math::SurfacePatch> patch_obj,
    const std::string& varname_patch_obj,
    const std::string& body_rob,
    std::shared_ptr<const math::SurfacePatch> patch_rob,
    const std::string& varname_patch_rob,
    double tolerance_p,
    double tolerance_n)
  : ik::Constraint(5, name, {varname_patch_obj, varname_patch_rob, "pose"}),
    m_rbtree(rbtree),
    m_body_obj(m_rbtree->get_body(body_obj)),
    m_body_rob(m_rbtree->get_body(body_rob)),
    m_patch_obj(patch_obj),
    m_patch_rob(patch_rob),
    m_varname_patch_obj(varname_patch_obj),
    m_varname_patch_rob(varname_patch_rob) {
  m_bounds.resize(5);
  for (size_t i = 0; i < 3; i++) {
    m_bounds.at(i) = optimization::Bounds(-tolerance_p, tolerance_p);
  }
  m_bounds.at(3) = optimization::Bounds(-tolerance_n, tolerance_n);
  m_bounds.at(4) = optimization::Bounds(-tolerance_n, tolerance_n);

  m_pose_addr = m_body_obj->get_support_qpos_addr();
}

//==============================================================================
math::VectorXd BodyContactConstraint::get_values(
    const math::VectorXd& x) const {
  math::Vector2d uv_obj, uv_rob;
  math::VectorXd pose;

  std::tie(uv_obj, uv_rob, pose) = parse_dependent_vars(x);
  pose.segment(3, 4)             = pose.segment(3, 4).normalized();

  math::VectorXd qpos                              = math::VectorXd(m_qpos);
  qpos.segment(m_pose_addr[0], m_pose_addr.size()) = pose;
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  // position of the contact point should be the same in the world frame
  math::Vector3d bpos_obj = m_patch_obj->evaluate(uv_obj);
  math::Vector3d bpos_rob = m_patch_rob->evaluate(uv_rob);

  math::Vector3d xpos_obj = m_body_obj->get_xpos(tforms, bpos_obj);
  math::Vector3d xpos_rob = m_body_rob->get_xpos(tforms, bpos_rob);

  math::Matrix3d xmat_obj = m_body_obj->get_xmat(tforms);
  math::Matrix3d xmat_rob = m_body_rob->get_xmat(tforms);

  // orientation of the normal should be in the opposite direction
  math::Vector3d btan_u_obj = m_patch_obj->jacb(uv_obj).col(0);
  math::Vector3d btan_v_obj = m_patch_obj->jacb(uv_obj).col(1);
  math::Vector3d xtan_u_obj = xmat_obj * btan_u_obj;
  math::Vector3d xtan_v_obj = xmat_obj * btan_v_obj;

  math::Vector3d bnorm_rob = m_patch_rob->normal(uv_rob);
  // bnorm_rob.normalize();
  math::Vector3d xnorm_rob = xmat_rob * bnorm_rob;

  math::VectorXd ret(get_rows());
  ret.segment(0, 3) = xpos_obj - xpos_rob;
  ret.segment(3, 1) = xtan_u_obj.transpose() * xnorm_rob;
  ret.segment(4, 1) = xtan_v_obj.transpose() * xnorm_rob;

  return ret;
}

//==============================================================================
optimization::VecBound BodyContactConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void BodyContactConstraint::fill_jacobian_block(
    Variable::ConstPtr varset, math::SparseJacobian& jac_block) const {

  math::Vector2d uv_obj, uv_rob;
  math::VectorXd pose;
  std::tie(uv_obj, uv_rob, pose)
      = parse_dependent_vars(get_dependent_variable_values());

  pose.segment(3, 4) = pose.segment(3, 4).normalized();

  math::VectorXd qpos = math::VectorXd(m_qpos);

  qpos.segment(m_pose_addr[0], m_pose_addr.size()) = pose;

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix3d xmat_obj = m_body_obj->get_xmat(tforms);
  math::Matrix3d xmat_rob = m_body_rob->get_xmat(tforms);

  math::Matrix5X cjacb;
  if (varset->get_name() == "pose") {
    cjacb = math::Matrix5X::Zero(5, 7);

    // first 3 rows for position
    math::Vector3d bpos_obj = m_patch_obj->evaluate(uv_obj);
    math::Vector3d xpos_obj = xmat_obj * bpos_obj;
    math::Matrix34 J_r = math::Quaternion(pose[3], pose[4], pose[5], pose[6])
                             .diff_omega_coeffs();

    math::Matrix3X J_1    = math::Matrix3X::Zero(3, 7);
    J_1.block<3, 4>(0, 3) = J_r;
    math::Matrix3X J_2    = math::Matrix3X::Zero(3, 7);
    J_2.block<3, 3>(0, 0) = math::Matrix3X::Identity(3, 3);

    // last 2 rows for opposite vec
    // c_2,1
    math::Vector3d bnorm_rob = m_patch_rob->normal(uv_rob);
    // bnorm_rob.normalize();
    math::Vector3d xnorm_rob  = xmat_rob * bnorm_rob;
    math::Vector3d btan_u_obj = m_patch_obj->jacb(uv_obj).col(0);
    math::Vector3d xtan_u_obj = xmat_obj * btan_u_obj;

    // c_2,2
    math::Vector3d btan_v_obj = m_patch_obj->jacb(uv_obj).col(1);
    math::Vector3d xtan_v_obj = xmat_obj * btan_v_obj;

    cjacb.block<3, 7>(0, 0) = -xpos_obj.skew() * J_1 + J_2;
    cjacb.block<1, 4>(3, 3)
        = -xnorm_rob.transpose() * (xtan_u_obj.skew() * J_r);
    cjacb.block<1, 4>(4, 3)
        = -xnorm_rob.transpose() * (xtan_v_obj.skew() * J_r);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_patch_obj) {
    cjacb = math::Matrix5X(5, 2);

    math::Vector3d bnorm_rob            = m_patch_rob->normal(uv_rob);
    math::Vector3d xnorm_rob            = xmat_rob * bnorm_rob;
    math::array2<math::Vector3d> derivs = m_patch_obj->derivatives(2, uv_obj);
    math::Matrix32 dsu_obj;
    dsu_obj.block<3, 1>(0, 0) = derivs(2, 0);
    dsu_obj.block<3, 1>(0, 1) = derivs(1, 1);

    math::Matrix32 dsv_obj;
    dsv_obj.block<3, 1>(0, 0) = derivs(1, 1);
    dsv_obj.block<3, 1>(0, 1) = derivs(0, 2);

    // top 3 rows for position
    cjacb.block<3, 2>(0, 0) = xmat_obj * m_patch_obj->jacb(uv_obj);
    // bottom 2 rows for opposite vec
    cjacb.block<1, 2>(3, 0) = xnorm_rob.transpose() * (xmat_obj * dsu_obj);
    cjacb.block<1, 2>(4, 0) = xnorm_rob.transpose() * (xmat_obj * dsv_obj);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
  else if (varset->get_name() == m_varname_patch_rob) {
    cjacb = math::Matrix5X(5, 2);

    math::Matrix32 s_jacb     = m_patch_obj->jacb(uv_obj);
    math::Vector3d xtan_u_obj = xmat_obj * s_jacb.col(0);
    math::Vector3d xtan_v_obj = xmat_obj * s_jacb.col(1);

    math::Matrix32 norm_jacb = m_patch_rob->normal_jacb(uv_rob);

    // top 3 rows for position
    cjacb.block<3, 2>(0, 0) = -xmat_rob * m_patch_rob->jacb(uv_rob);
    // bottom 2 rows for opposite vec
    cjacb.block<1, 2>(3, 0) = xtan_u_obj.transpose() * (xmat_rob * norm_jacb);
    cjacb.block<1, 2>(4, 0) = xtan_v_obj.transpose() * (xmat_rob * norm_jacb);

    for (int i = 0; i < cjacb.rows(); i++) {
      for (int j = 0; j < cjacb.cols(); j++) {
        jac_block.coeffRef(i, j) = cjacb(i, j);
      }
    }
  }
}

//==============================================================================
std::tuple<math::Vector2d, math::Vector2d, math::VectorXd>
BodyContactConstraint::parse_dependent_vars(const math::VectorXd& x) const {
  CROBOT_ASSERT_DEBUG(x.size() == 2 + 2 + 7, "Variable dimension mismatch");

  // [u1, v1], [u2, v2], [pose]
  math::Vector2d uv_obj = x.segment(0, 2);
  math::Vector2d uv_rob = x.segment(2, 2);
  math::VectorXd pose   = x.segment(4, 7);
  return std::make_tuple(uv_obj, uv_rob, pose);
}

//==============================================================================
void BodyContactConstraint::set_qpos(const math::VectorXd& qpos) {
  m_qpos = qpos;
}
