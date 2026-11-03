#include "crobot/planning/body_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
BodyTrajectory::BodyTrajectory(kdl::ConstRigidBodyPtr body, double duration)
  : Trajectory(), m_body(body), m_duration(duration) {
  // TODO
}

//==============================================================================
double BodyTrajectory::get_duration() const {
  return m_duration;
}

//==============================================================================
kdl::ConstRigidBodyPtr BodyTrajectory::get_body() const {
  return m_body;
}
