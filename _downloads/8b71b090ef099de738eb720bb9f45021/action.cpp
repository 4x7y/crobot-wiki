#include "crobot/control/action.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
Action::Action(const std::string& name, double duration)
  : m_duration(duration), m_name(name) {
  CROBOT_ASSERT_ERROR(duration > 0.0, "duration must be positive.");
}

//==============================================================================
double Action::get_duration() const {
  return m_duration;
}

//==============================================================================
const std::string& Action::get_name() const {
  return m_name;
}

//==============================================================================
void Action::set_name(const std::string& name) {
  m_name = name;
}

//==============================================================================
bool Action::is_inited() const {
  return m_is_inited;
}

//==============================================================================
bool Action::is_active() const {
  return m_is_active;
}

//==============================================================================
Action::ActionType Action::get_type() const {
  return m_type;
}

//==============================================================================
void Action::set_dt(double dt) {
  m_dt = dt;
}
