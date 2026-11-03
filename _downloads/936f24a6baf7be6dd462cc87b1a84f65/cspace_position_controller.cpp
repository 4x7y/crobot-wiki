#include <iostream>

#include <yaml-cpp/yaml.h>

#include "crobot/common/assert.hpp"
#include "crobot/control/cspace_position_controller.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
CSpacePositionControllerPtr CSpacePositionController::MakeShared(
    kdl::ConstRigidBodyTreePtr robot,
    math::VectorXd active_id,
    const std::string& params_file) {
  return std::make_shared<CSpacePositionController>(
      std::move(robot), std::move(active_id), params_file);
}

//==============================================================================
CSpacePositionController::CSpacePositionController(
    kdl::ConstRigidBodyTreePtr robot,
    math::VectorXd active_id,
    const std::string& params_file)
  : m_robot(std::move(robot)), m_active_id(std::move(active_id)) {
  CROBOT_ASSERT_ERROR(m_robot);

  YAML::Node ctrl_params = YAML::LoadFile(params_file);

  std::vector<double> kp = ctrl_params["KPv"].as<std::vector<double>>();
  std::vector<double> kd = ctrl_params["KDv"].as<std::vector<double>>();
  std::vector<double> ki = ctrl_params["KIv"].as<std::vector<double>>();

  m_nq        = m_robot->get_qpos_dim();
  m_nq_active = m_active_id.size();

  CROBOT_ASSERT_ERROR(static_cast<int>(kp.size()) == m_nq_active);

  m_kp = math::MatrixXd::Identity(m_nq_active, m_nq_active);
  m_kd = math::MatrixXd::Identity(m_nq_active, m_nq_active);
  m_ki = math::MatrixXd::Identity(m_nq_active, m_nq_active);

  m_kp.diagonal() = math::VectorXd::Map(kp.data(), kp.size());
  m_kd.diagonal() = math::VectorXd::Map(kd.data(), kd.size());
  m_ki.diagonal() = math::VectorXd::Map(ki.data(), ki.size());

  reset();
}

//==============================================================================
void CSpacePositionController::reset() {
  m_time_last_update = -1;
  m_qpos_error_int   = math::VectorXd::Zero(m_nq_active);
}

//==============================================================================
void CSpacePositionController::set_qpos_desired(const math::VectorXd& qpos) {
  CROBOT_ASSERT_ERROR(qpos.size() == m_nq_active);
  m_qpos_desired = qpos;
}

//==============================================================================
void CSpacePositionController::set_qvel_desired(const math::VectorXd& qvel) {
  CROBOT_ASSERT_ERROR(qvel.size() == m_nq_active);
  m_qvel_desired = qvel;
}

//==============================================================================
void CSpacePositionController::set_qacc_desired(const math::VectorXd& qacc) {
  CROBOT_ASSERT_ERROR(qacc.size() == m_nq_active);
  m_qacc_desired = qacc;
}

//==============================================================================
math::VectorXd CSpacePositionController::compute(
    const math::VectorXd& qpos_all,
    const math::VectorXd& qvel_all,
    double time) {
  CROBOT_ASSERT_ERROR(qpos_all.size() == m_nq);
  CROBOT_ASSERT_ERROR(qvel_all.size() == m_nq);

  math::VectorXd qpos = qpos_all(m_active_id);
  math::VectorXd qvel = qvel_all(m_active_id);

  // Initialize m_time_last_update if this function is called for the first
  // time.
  if (m_time_last_update < 0) {
    m_time_last_update = time;
  }
  double dt          = time - m_time_last_update;
  m_time_last_update = time;

  // Update the integration of the joint position error.
  m_qpos_error_int += dt * (m_qpos_desired - qpos);

  math::VectorXd qacc = m_qacc_desired + m_kp * (m_qpos_desired - qpos)
                        + m_kd * (m_qvel_desired - qvel)
                        + m_ki * m_qpos_error_int;

  math::VectorXd qacc_all = math::VectorXd::Zero(m_nq);
  qacc_all(m_active_id)   = qacc;

  // std::cout << "qacc: " << qacc.transpose() << std::endl;

  math::VectorXd qfrc_resolved
      = m_robot->inverse_dynamics(qpos_all, qvel_all, qacc_all);
  return qfrc_resolved;
}
