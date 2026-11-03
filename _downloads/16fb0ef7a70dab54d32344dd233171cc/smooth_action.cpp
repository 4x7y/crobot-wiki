#include "crobot/common/logging.hpp"
#include "crobot/control/smooth_action.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
SmoothAction::SmoothAction(
    const std::string& name,
    double duration,
    Action::Ptr action_prev,
    Action::Ptr action_next)
  : Action(name, duration),
    m_action_prev(action_prev),
    m_action_next(action_next) {
}

//==============================================================================
SmoothAction::SmoothAction(
    const std::string& name,
    double duration,
    math::VectorXd qpos,
    Action::Ptr action_next)
  : Action(name, duration),
    m_action_prev(nullptr),
    m_action_next(action_next),
    m_qpos_actual_prev(qpos),
    m_qpos_command_prev(qpos) {
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> SmoothAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  double c1 = 1.0 - time / get_duration();
  double c2 = 1.0 - c1;

  math::VectorXd qpos_actual_next;
  math::VectorXd qpos_command_next;
  std::tie(qpos_actual_next, qpos_command_next)
      = m_action_next->get_command(time, qpos_actual, qpos_cmd, qvel_actual);

  if (!m_action_next->is_active()) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  return std::make_pair(
      c1 * m_qpos_actual_prev + c2 * qpos_actual_next,
      c1 * m_qpos_command_prev + c2 * qpos_command_next);
}

//==============================================================================
void SmoothAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  CROBOT_LOG_INFO("Initializing SmoothAction: {}", get_name());
  m_is_inited = true;
  m_is_active = true;
  m_type      = SMOOTH;
  if (m_action_prev) {
    // std::tie(m_qpos_actual_prev, m_qpos_command_prev)
    //     = m_action_prev->get_command(
    //         m_action_prev->get_duration(), qpos_actual);

    m_qpos_actual_prev  = qpos_actual;
    m_qpos_command_prev = qpos_cmd;
  }

  m_action_next->initialize(qpos_actual, qpos_cmd);
  // std::tie(m_qpos_actual_next, m_qpos_command_next)
  //     = m_action_next->get_command(0.0, qpos_actual, qpos_cmd);
  // CROBOT_LOG_INFO("m_qpos_actual_prev  = {}",
  // m_qpos_actual_prev.transpose()); CROBOT_LOG_INFO("m_qpos_command_prev =
  // {}", m_qpos_command_prev.transpose()); CROBOT_LOG_INFO("m_qpos_actual_next
  // = {}", m_qpos_actual_next.transpose());
  // CROBOT_LOG_INFO("m_qpos_command_next = {}",
  // m_qpos_command_next.transpose()); CROBOT_LOG_INFO("Completed initializing
  // SmoothAction: {}", get_name());
}

//==============================================================================
void SmoothAction::fix_joint(const std::string& joint_name) {
  (void)joint_name;
}

//==============================================================================
void SmoothAction::fix_joint(const std::string& joint_name, double value) {
  (void)joint_name;
  (void)value;
}

//==============================================================================
void SmoothAction::fix_body(const std::string& body_name) {
  (void)body_name;
}

//==============================================================================
kdl::Grasp::ConstPtr SmoothAction::get_grasp() const {
  return m_action_next->get_grasp();
}
//==============================================================================
void SmoothAction::set_null_space_obj([
    [maybe_unused]] ik::NullSpaceObj::Ptr obj) {
  // DO NOTHING
}
