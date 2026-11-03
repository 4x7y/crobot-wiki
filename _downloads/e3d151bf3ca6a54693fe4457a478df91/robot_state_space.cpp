#include "crobot/planning/robot_state_space.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RobotStateSpace::RobotStateSpace(kdl::ConstRigidBodyTreePtr rbtree)
  : StateSpace(), m_rbtree(rbtree) {
}

//==============================================================================
math::VectorXd RobotStateSpace::sample() const {
  return m_rbtree->make_random_qpos();
}

//==============================================================================
math::VectorXd RobotStateSpace::interpolate(
    const math::VectorXd& source,
    const math::VectorXd& target,
    double ratio) const {
  return source + (target - source) * ratio;
}

//==============================================================================
double RobotStateSpace::distance(
    const math::VectorXd& from, const math::VectorXd& to) const {
  return (to - from).norm();
}

//==============================================================================
bool RobotStateSpace::is_state_valid(const math::VectorXd& state) const {
  (void)state;
  return true;
}

//==============================================================================
bool RobotStateSpace::is_transition_valid(
    const math::VectorXd& from, const math::VectorXd& to) const {
  (void)from;
  (void)to;
  return true;
}
