#include "crobot/ik/cost_body_pose_potential.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
BodyPosePotentialCost::BodyPosePotentialCost(
    const std::string& name, const math::VectorXd& pose_est)
  : ik::Cost(name, {"pose"}) {
  m_xpos_est  = pose_est.segment(0, 3);
  m_xquat_est = pose_est.segment(3, 4).normalized();
}

//==============================================================================
double BodyPosePotentialCost::get_cost(const math::VectorXd& pose) const {

  double temp
      = pose.segment(3, 4).normalized().transpose() * m_xquat_est.normalized();
  double angerr = 1.0 - std::abs(temp);
  return angerr;
}

//==============================================================================
void BodyPosePotentialCost::fill_jacobian_block(
    Variable::ConstPtr varsets, math::SparseJacobian& jac_block) const {
  if (varsets->get_name() == "pose") {
    jac_block = get_jacobian_finite_diff();
  }

  // if (varsets->get_name() == "pose") {
  //   math::VectorXd pose = get_dependent_variable_values();

  //   math::Vector3d posdiff = pose.segment(0, 3) - m_xpos_est;
  //   math::Vector3d df1_dp  = 2 * posdiff;
  //   math::Vector4d df1_dr  = math::Vector4d::Zero();

  //   math::Vector4d quat     = pose.segment(3, 4).normalized();
  //   math::Vector4d quat_est = m_xquat_est.normalized();

  //   double dot_prod = quat.transpose() * quat_est;

  //   math::Vector3d df2_dp = math::Vector3d::Zero();
  //   math::Vector4d df2_dr = -(dot_prod / (std::abs(dot_prod))) * quat_est;

  //   // set jac_block
  //   for (std::size_t i = 0; i < 3; i++) {
  //     jac_block.coeffRef(0, i)
  //         = m_weight_f1 * df1_dp[i] + m_weight_f2 * df2_dp[i];
  //   }

  //   for (std::size_t i = 3; i < 7; i++) {
  //     jac_block.coeffRef(0, i)
  //         = m_weight_f1 * df1_dr[i - 3] + m_weight_f2 * df2_dr[i - 3];
  //   }
  // }
}

//==============================================================================
void BodyPosePotentialCost::set_body_xpos_est(
    const math::Vector3d& xpos, const math::Quaternion& xquat) {
  m_xpos_est  = xpos;
  m_xquat_est = xquat.as_vector();
}

//==============================================================================
void BodyPosePotentialCost::set_body_xpos_est(const math::VectorXd& pose) {
  m_xpos_est  = pose.segment(0, 3);
  m_xquat_est = pose.segment(3, 4).normalized();
}
