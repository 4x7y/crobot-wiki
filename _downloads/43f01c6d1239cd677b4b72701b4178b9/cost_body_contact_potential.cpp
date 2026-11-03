#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/ik/cost_body_contact_potential.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
BodyContactPotentialCost::BodyContactPotentialCost(
    const std::string& name,
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_obj,
    std::shared_ptr<const math::SurfacePatch> patch_obj,
    const std::string& varname_patch_obj,
    const std::string& body_rob,
    std::shared_ptr<const math::SurfacePatch> patch_rob,
    const std::string& varname_patch_rob,
    const math::Vector3d& xpos_screw)
  : ik::Cost(name, {varname_patch_obj, varname_patch_rob, "pose", "lagrange"}),
    m_rbtree(rbtree),
    m_body_obj(m_rbtree->get_body(body_obj)),
    m_body_rob(m_rbtree->get_body(body_rob)),
    m_patch_obj(patch_obj),
    m_patch_rob(patch_rob),
    m_varname_patch_obj(varname_patch_obj),
    m_varname_patch_rob(varname_patch_rob),
    m_xpos_screw(xpos_screw) {
  m_pose_addr = m_body_obj->get_support_qpos_addr();
}

//==============================================================================
double BodyContactPotentialCost::get_cost(const math::VectorXd& x) const {

  // std::cout << "get cost start" << std::endl;

  math::Vector2d uv_obj, uv_rob;
  math::VectorXd pose;
  math::VectorXd lagrange_multipliers;

  std::tie(uv_obj, uv_rob, pose, lagrange_multipliers)
      = parse_dependent_vars(x);

  double l_p  = lagrange_multipliers[0];
  double l_ru = lagrange_multipliers[1];
  double l_rv = lagrange_multipliers[2];
  double l_s  = lagrange_multipliers[3];

  pose.segment(3, 4) = pose.segment(3, 4).normalized();

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

  math::VectorXd ret(6);
  ret.segment(0, 3) = xpos_obj - xpos_rob;
  ret.segment(3, 1) = xtan_u_obj.transpose() * xnorm_rob;
  ret.segment(4, 1) = xtan_v_obj.transpose() * xnorm_rob;

  math::Vector3d p_screw_diff = pose.segment(0, 3) - m_xpos_screw;

  // std::cout << "get cost end" << std::endl;

  return l_p * ret.segment(0, 3).squaredNorm() + l_ru * (ret[3] * ret[3])
         + l_rv * (ret[4] * ret[4]) + l_s * p_screw_diff.squaredNorm();
}

//==============================================================================
void BodyContactPotentialCost::fill_jacobian_block(
    Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const {
  math::Vector2d uv_obj, uv_rob;
  math::VectorXd pose;
  math::VectorXd lagrange_multipliers;

  std::tie(uv_obj, uv_rob, pose, lagrange_multipliers)
      = parse_dependent_vars(get_dependent_variable_values());

  double l_p  = lagrange_multipliers[0];
  double l_ru = lagrange_multipliers[1];
  double l_rv = lagrange_multipliers[2];
  double l_s  = lagrange_multipliers[3];

  pose.segment(3, 4) = pose.segment(3, 4).normalized();

  math::VectorXd qpos = math::VectorXd(m_qpos);

  qpos.segment(m_pose_addr[0], m_pose_addr.size()) = pose;

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Matrix3d xmat_obj = m_body_obj->get_xmat(tforms);
  math::Matrix3d xmat_rob = m_body_rob->get_xmat(tforms);

  math::Vector3d bpos_obj = m_patch_obj->evaluate(uv_obj);
  math::Vector3d xpos_obj = xmat_obj * bpos_obj;
  math::Vector3d bpos_rob = m_patch_rob->evaluate(uv_rob);

  math::Vector3d bnorm_rob  = m_patch_rob->normal(uv_rob);
  math::Vector3d xnorm_rob  = xmat_rob * bnorm_rob;
  math::Vector3d btan_u_obj = m_patch_obj->jacb(uv_obj).col(0);
  math::Vector3d xtan_u_obj = xmat_obj * btan_u_obj;
  math::Vector3d btan_v_obj = m_patch_obj->jacb(uv_obj).col(1);
  math::Vector3d xtan_v_obj = xmat_obj * btan_v_obj;

  math::Vector3d xxpos_obj = m_body_obj->get_xpos(tforms, bpos_obj);
  math::Vector3d xxpos_rob = m_body_rob->get_xpos(tforms, bpos_rob);

  math::Vector3d ctc_xpos_diff = xxpos_obj - xxpos_rob;
  double x_u                   = xtan_u_obj.transpose() * xnorm_rob;
  double x_v                   = xtan_v_obj.transpose() * xnorm_rob;
  math::Vector3d p_screw_diff  = pose.segment(0, 3) - m_xpos_screw;

  if (varsets->get_name() == "pose") {

    // first 3 rows for position
    math::Matrix34 J_r = math::Quaternion(pose[3], pose[4], pose[5], pose[6])
                             .diff_omega_coeffs();

    math::Matrix3X J_1    = math::Matrix3X::Zero(3, 7);
    J_1.block<3, 4>(0, 3) = J_r;
    math::Matrix3X J_2    = math::Matrix3X::Zero(3, 7);
    J_2.block<3, 3>(0, 0) = math::Matrix3X::Identity(3, 3);

    math::VectorXd Jac_p
        = 2 * (-xpos_obj.skew() * J_1 + J_2).transpose() * ctc_xpos_diff;
    math::VectorXd Jac_r1
        = -2 * x_u * (xtan_u_obj.skew() * J_r).transpose() * xnorm_rob;
    math::VectorXd Jac_r2
        = -2 * x_v * (xtan_v_obj.skew() * J_r).transpose() * xnorm_rob;

    math::VectorXd stack(7);
    stack << p_screw_diff[0], p_screw_diff[1], p_screw_diff[2], 0, 0, 0, 0;
    math::VectorXd Jac_sc = 2 * stack;

    math::VectorXd Jac
        = l_p * Jac_p + l_ru * Jac_r1 + l_rv * Jac_r2 + l_s * Jac_sc;

    for (int i = 0; i < Jac.size(); i++) {
      jac_block.coeffRef(0, i) = Jac[i];
    }
  }
  else if (varsets->get_name() == m_varname_patch_obj) {

    math::array2<math::Vector3d> derivs = m_patch_obj->derivatives(2, uv_obj);
    math::Matrix32 dsu_obj;
    dsu_obj.block<3, 1>(0, 0) = derivs(2, 0);
    dsu_obj.block<3, 1>(0, 1) = derivs(1, 1);

    math::Matrix32 dsv_obj;
    dsv_obj.block<3, 1>(0, 0) = derivs(1, 1);
    dsv_obj.block<3, 1>(0, 1) = derivs(0, 2);

    // top 3 rows for position
    math::Vector2d Jac_p = 2
                           * (xmat_obj * m_patch_obj->jacb(uv_obj)).transpose()
                           * ctc_xpos_diff;
    // bottom 2 rows for opposite vec
    math::Vector2d Jac_r1
        = 2 * x_u * (xmat_obj * dsu_obj).transpose() * xnorm_rob;

    math::Vector2d Jac_r2
        = 2 * x_v * (xmat_obj * dsv_obj).transpose() * xnorm_rob;

    math::Vector2d Jac_sc(0, 0);

    math::VectorXd Jac
        = l_p * Jac_p + l_ru * Jac_r1 + l_rv * Jac_r2 + l_s * Jac_sc;

    for (int i = 0; i < Jac.size(); i++) {
      jac_block.coeffRef(0, i) = Jac[i];
    }
  }
  else if (varsets->get_name() == m_varname_patch_rob) {

    math::Matrix32 norm_jacb = m_patch_rob->normal_jacb(uv_rob);

    math::Vector2d Jac_p = 2
                           * (-xmat_rob * m_patch_rob->jacb(uv_rob)).transpose()
                           * ctc_xpos_diff;
    math::Vector2d Jac_r1
        = 2 * x_u * (xmat_rob * norm_jacb).transpose() * xtan_u_obj;

    math::Vector2d Jac_r2
        = 2 * x_v * (xmat_rob * norm_jacb).transpose() * xtan_v_obj;

    math::Vector2d Jac_sc(0, 0);

    math::VectorXd Jac
        = l_p * Jac_p + l_ru * Jac_r1 + l_rv * Jac_r2 + l_s * Jac_sc;

    for (int i = 0; i < Jac.size(); i++) {
      jac_block.coeffRef(0, i) = Jac[i];
    }
  }
  else if (varsets->get_name() == "lagrange") {
    jac_block.coeffRef(0, 0) = ctc_xpos_diff.squaredNorm();
    jac_block.coeffRef(0, 1) = x_u * x_u;
    jac_block.coeffRef(0, 2) = x_v * x_v;
    jac_block.coeffRef(0, 3) = p_screw_diff.squaredNorm();
  }
}

//==============================================================================
std::tuple<math::Vector2d, math::Vector2d, math::VectorXd, math::VectorXd>
BodyContactPotentialCost::parse_dependent_vars(const math::VectorXd& x) const {
  CROBOT_ASSERT_DEBUG(x.size() == 2 + 2 + 7 + 4, "Variable dimension mismatch");

  // [u1, v1], [u2, v2], [pose]
  math::Vector2d uv_obj   = x.segment(0, 2);
  math::Vector2d uv_rob   = x.segment(2, 2);
  math::VectorXd pose     = x.segment(4, 7);
  math::VectorXd lagrange = x.segment(11, x.size() - 11);
  return std::make_tuple(uv_obj, uv_rob, pose, lagrange);
}

//==============================================================================
void BodyContactPotentialCost::set_qpos(const math::VectorXd& qpos) {
  m_qpos = qpos;
}
//==============================================================================
void BodyContactPotentialCost::set_screw_xpos(
    const math::Vector3d& xpos_screw) {
  m_xpos_screw = xpos_screw;
}
