#include "crobot/planning/rrt_control.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RDT::RDT(StatePropagator::Ptr propagator, GoalReachFunctionPtr goal_reach_func)
  : m_state_propagator(std::move(propagator)),
    m_goal_reach_func(std::move(goal_reach_func)) {
}

//==============================================================================
ControlPath RDT::plan(const math::VectorXd& start, const math::VectorXd& goal) {
  (void)start;
  (void)goal;
  return ControlPath();
}

//==============================================================================
void RDT::initialize(const math::VectorXd& start, const math::VectorXd& goal) {
  (void)start;
  (void)goal;
}

//==============================================================================
void RDT::generate_outputs() {
}
