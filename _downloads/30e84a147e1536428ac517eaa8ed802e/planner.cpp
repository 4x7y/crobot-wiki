
#include "crobot/math/state_space.hpp"
#include "crobot/planning/planner.hpp"
#include "crobot/planning/viewer.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
Planner::Planner(Planner::ConstStateSpacePtr state_space)
  : m_duration(std::chrono::steady_clock::duration::max()),
    m_state_space(state_space),
    m_viewer(nullptr),
    m_time() {
}

//==============================================================================
Planner::~Planner() {
}

//==============================================================================
::std::chrono::steady_clock::duration Planner::get_duration() const {
  return m_duration;
}

//==============================================================================
Planner::ConstStateSpacePtr Planner::get_state_space() const {
  return m_state_space;
}

//==============================================================================
std::shared_ptr<Viewer> Planner::get_viewer() const {
  return m_viewer;
}

//==============================================================================
void Planner::set_duration(
    const ::std::chrono::steady_clock::duration& duration) {
  m_duration = duration;
}

//==============================================================================
void Planner::set_viewer(std::shared_ptr<Viewer> viewer) {
  m_viewer = viewer;
}
