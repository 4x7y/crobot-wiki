#include "crobot/control/admittance_controller.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
AdmittanceController::AdmittanceController(kdl::ConstRigidBodyTreePtr rbtree)
  : m_rbtree(rbtree) {
  CROBOT_ASSERT_ERROR(m_rbtree != nullptr, "rbtree is null");
  m_ik_actual  = std::make_shared<ik::JacobianIK>(m_rbtree);
  m_ik_command = std::make_shared<ik::JacobianIK>(m_rbtree);
}

//==============================================================================
bool AdmittanceController::compute(
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel) {
  if (!m_ik_actual->solve(qpos_actual, qvel)) {
    CROBOT_LOG_ERROR("Failed to solve actual position in admittance control.");
    return false;
  }

  if (!m_ik_command->solve(qpos_cmd, qvel)) {
    CROBOT_LOG_ERROR("Failed to solve command position in admittance control.");
    return false;
  }

  return true;
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> AdmittanceController::get_result() {
  return std::make_pair<>(m_ik_actual->get_qpos(), m_ik_command->get_qpos());
}

//==============================================================================
AdmittanceControlTarget::Ptr AdmittanceController::add_target(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    math::EulerAngleSeq euler_seq,
    const std::vector<int>& dofs,
    kdl::BodyContact::ConstPtr contact) {
  kdl::ConstRigidBodyPtr body = m_rbtree->get_body(body_name);
  AdmittanceControlTarget::Ptr target_without_contact
      = std::make_shared<AdmittanceControlTarget>(
          desired_frame, contact, false);
  m_ik_actual->add_target_selected_dofs(
      target_without_contact, body_name, dofs, euler_seq, bpos, rotm);
  AdmittanceControlTarget::Ptr target
      = std::make_shared<AdmittanceControlTarget>(desired_frame, contact, true);
  m_ik_command->add_target_selected_dofs(
      target, body_name, dofs, euler_seq, bpos, rotm);
  return target;
}

//==============================================================================
AdmittanceControlTarget::Ptr AdmittanceController::add_rolling_target(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    math::EulerAngleSeq euler_seq,
    const std::vector<int>& dofs,
    kdl::BodyRollingContact::ConstPtr contact) {
  kdl::ConstRigidBodyPtr body = m_rbtree->get_body(body_name);
  AdmittanceControlTarget::Ptr target_without_contact
      = std::make_shared<AdmittanceControlTarget>(
          desired_frame, contact, false);
  m_ik_actual->add_rolling_target_selected_dofs(
      target_without_contact, body_name, dofs, contact, euler_seq);
  AdmittanceControlTarget::Ptr target
      = std::make_shared<AdmittanceControlTarget>(desired_frame, contact, true);
  m_ik_command->add_rolling_target_selected_dofs(
      target, body_name, dofs, contact, euler_seq);
  return target;
}

//==============================================================================
void AdmittanceController::fix_joint(
    const std::string& joint_name, double value) {
  m_ik_actual->fix_joint(joint_name, value);
  m_ik_command->fix_joint(joint_name, value);
}

//==============================================================================
void AdmittanceController::fix_joint(const std::string& joint_name) {
  m_ik_actual->fix_joint(joint_name);
  m_ik_command->fix_joint(joint_name);
}

//==============================================================================
void AdmittanceController::fix_body(const std::string& body_name) {
  m_ik_actual->fix_body(body_name);
  m_ik_command->fix_body(body_name);
}

//==============================================================================
void AdmittanceController::set_null_space_objective(ik::NullSpaceObj::Ptr obj) {
  m_ik_actual->set_null_space_objective(obj);
  m_ik_command->set_null_space_objective(obj);
}
