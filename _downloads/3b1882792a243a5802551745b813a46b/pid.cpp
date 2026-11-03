#include "crobot/control/pid.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
PIDController::PIDController(
    const math::VectorXd& kp,
    const math::VectorXd& ki,
    const math::VectorXd& kd,
    std::function<math::VectorXd()> qpos_callback,
    std::function<math::VectorXd()> qvel_callback)
  : m_kp(kp),
    m_ki(ki),
    m_kd(kd),
    m_qpos_callback(qpos_callback),
    m_qvel_callback(qvel_callback),
    m_qpos_desired(math::VectorXd::Zero(kp.size())),
    m_qvel_desired(math::VectorXd::Zero(kp.size())),
    m_qpos_err_int(math::VectorXd::Zero(kp.size())),
    m_gain(math::VectorXd::Zero(kp.size())) {
  CROBOT_ASSERT_FATAL(kp.size() == kd.size());
  CROBOT_ASSERT_FATAL(kp.size() == ki.size());
}

//==============================================================================
void PIDController::set_desired(
    const math::VectorXd& qpos, const math::VectorXd& qvel) {
  CROBOT_ASSERT_FATAL(m_kp.size() == qpos.size());
  CROBOT_ASSERT_FATAL(m_kp.size() == qvel.size());
  m_qpos_desired = qpos;
  m_qvel_desired = qvel;
}

//==============================================================================
void PIDController::update(double time, double duration) {
  (void)time;
  CROBOT_ASSERT_FATAL(duration > 0.0);

  math::VectorXd qpos = m_qpos_callback();
  math::VectorXd qvel = m_qvel_callback();

  math::VectorXd qpos_err = m_qpos_desired - qpos;
  math::VectorXd qvel_err = m_qvel_desired - qvel;
  m_qpos_err_int += duration * qpos_err;

  math::VectorXd qpos_gain         = m_kp.cwiseProduct(qpos_err);
  math::VectorXd qvel_gain         = m_kd.cwiseProduct(qvel_err);
  math::VectorXd qvel_err_int_gain = m_ki.cwiseProduct(m_qpos_err_int);

  m_gain = qpos_gain + qvel_gain + qvel_err_int_gain;
}

//==============================================================================
math::VectorXd PIDController::generate() {
  return m_gain;
}
