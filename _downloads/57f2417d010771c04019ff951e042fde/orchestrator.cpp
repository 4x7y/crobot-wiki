#include "crobot/control/orchestrator.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/control/smooth_action.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
Orchestrator::Orchestrator(
    double time_step,
    const math::VectorXd& qpos_init,
    const math::VectorXd& qvel_init)
  : m_time(0.0),
    m_time_step(time_step),
    m_duration(0.0),
    m_qpos_actual(qpos_init),
    m_qpos_cmd(qpos_init),
    m_qvel(qvel_init) {
  CROBOT_ASSERT_ERROR(time_step > 0.0, "time_step must be positive.");
}

//==============================================================================
math::VectorXd Orchestrator::next(
    const math::VectorXd& qpos_actual, const math::VectorXd& qvel_actual) {
  CROBOT_ASSERT_ERROR(!m_actions.empty(), "No actions added.");

  // update qpos_actual and qvel_actual only if they are not empty
  if (qpos_actual.size() != 0) {
    CROBOT_ASSERT_ERROR(
        qpos_actual.size() == m_qpos_actual.size(), "qpos_actual size wrong");
    m_qpos_actual = qpos_actual;
  }

  if (qvel_actual.size() != 0) {
    CROBOT_ASSERT_ERROR(
        qvel_actual.size() == m_qvel.size(), "qvel_actual size wrong");
    m_qvel = qvel_actual;
  }

  double time = m_time;

  Action::Ptr action_todo = nullptr;
  for (Action::Ptr action : m_actions) {

    if (time < action->get_duration()) {
      action_todo = action;
      if (!action->is_inited()) {
        CROBOT_LOG_INFO("Initializing action: {}", action->get_name());
        action->initialize(m_qpos_actual, m_qpos_cmd);
        CROBOT_LOG_INFO(
            "Complete action initialization: {}", action->get_name());
      }
      break;
    }

    if (action->get_type() != Action::SMOOTH)
      time -= action->get_duration();
  }

  math::VectorXd qpos_actual_resolved;
  math::VectorXd qpos_command_resolved;
  if (action_todo) {
    // CROBOT_LOG_INFO(
    //     "Time {}s: action = {}, local_time = {}",
    //     m_time,
    //     action_todo->get_name(),
    //     time);
    std::tie(qpos_actual_resolved, qpos_command_resolved)
        = action_todo->get_command(time, m_qpos_actual, m_qpos_cmd, m_qvel);
  }
  else {
    CROBOT_ASSERT_WARNING(
        action_todo == nullptr,
        "Time is too large: %f. Repeat the last action command.",
        m_time);
    Action::Ptr action                                    = m_actions.back();
    std::tie(qpos_actual_resolved, qpos_command_resolved) = action->get_command(
        action->get_duration(), m_qpos_actual, m_qpos_cmd, m_qvel);
  }

  if (action_todo->is_active()) {
    m_time += m_time_step;
    m_qpos_actual  = qpos_actual_resolved;
    m_qpos_cmd     = qpos_command_resolved;
    m_grasp_active = action_todo->get_grasp();
    return qpos_command_resolved;
  }
  else {
    CROBOT_LOG_WARN(
        "Orchestrator is resetting. Action = {}", action_todo->get_name());
    reset();
    return m_qpos_cmd;
  }
}

//==============================================================================
void Orchestrator::add_action(Action::Ptr action_next, double smooth_time) {
  CROBOT_ASSERT_ERROR(action_next != nullptr, "action is nullptr.");
  CROBOT_ASSERT_ERROR(
      action_next->get_duration() > 0.0, "action duration must be positive.");

  CROBOT_ASSERT_ERROR(
      action_next->get_duration() > smooth_time,
      "action duration must be larger than smooth_time.");

  action_next->set_dt(m_time_step);

  // add smooth action
  if (m_actions.empty() && smooth_time > 0.0) {
    m_actions.push_back(std::make_shared<SmoothAction>(
        "smooth_" + action_next->get_name(),
        smooth_time,
        m_qpos_actual,
        action_next));
  }
  else if (smooth_time > 0.0) {
    Action::Ptr action_prev = m_actions.back();
    m_actions.push_back(std::make_shared<SmoothAction>(
        "smooth_" + action_prev->get_name() + "_" + action_next->get_name(),
        smooth_time,
        action_prev,
        action_next));
  }
  // m_duration += m_actions.back()->get_duration();
  // CROBOT_LOG_INFO(
  //     "Added action: {} - {}s",
  //     m_actions.back()->get_name(),
  //     m_actions.back()->get_duration());
  // CROBOT_LOG_INFO("Total duration up to now: {}s", m_duration);

  // add actual action
  m_actions.push_back(action_next);
  m_duration += action_next->get_duration();
  CROBOT_LOG_INFO(
      "Added action: {} - {}s",
      action_next->get_name(),
      action_next->get_duration());
  CROBOT_LOG_INFO("Total duration up to now: {}s", m_duration);
}

//==============================================================================
double Orchestrator::get_duration() const {
  return m_duration;
}

//==============================================================================
double Orchestrator::get_time() const {
  return m_time;
}

//==============================================================================
bool Orchestrator::has_next() const {
  return m_time < m_duration;
}

//==============================================================================
math::VectorXd Orchestrator::get_qpos() const {
  return m_qpos_cmd;
}

//==============================================================================
void Orchestrator::reset() {
  m_time     = 0.0;
  m_duration = 0.0;
  m_qvel.setZero();
  m_actions.clear();
}

//==============================================================================
kdl::Grasp::ConstPtr Orchestrator::get_grasp() const {
  return m_grasp_active;
}

//==============================================================================
void Orchestrator::set_obj_pose(
    int qpos_start, const math::VectorXd& qpos_obj) {
  CROBOT_ASSERT_ERROR(qpos_obj.size() == 7, "qpos_obj size must be 7.");
  m_qpos_cmd.segment<7>(qpos_start) = qpos_obj;
}
