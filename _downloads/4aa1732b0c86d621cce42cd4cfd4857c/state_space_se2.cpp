#include "crobot/common/assert.hpp"
#include "crobot/math/random.hpp"
#include "crobot/math/rotation.hpp"
#include "crobot/planning/state_space_se2.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
StateSpaceSE2::StateSpaceSE2(
    const math::Vector3d& lb,
    const math::Vector3d& ub,
    double weight_xy,
    double weight_theta)
  : StateSpace(),
    m_lb(lb),
    m_ub(ub),
    m_weight_xy(weight_xy),
    m_weight_theta(weight_theta) {
}

//==============================================================================
math::VectorXd StateSpaceSE2::sample() const {
  return math::Random::uniform(m_lb, m_ub);
}

//==============================================================================
math::VectorXd StateSpaceSE2::interpolate(
    const math::VectorXd& source,
    const math::VectorXd& target,
    double ratio) const {
  // Find difference between states
  math::VectorXd state_diff = target - source;

  // Restrict dTheta to [-pi pi]
  state_diff(2) = math::wrap_to_pi(state_diff(2));

  // Calculate interpolated states
  math::VectorXd interp_state = source + ratio * state_diff;

  // Restrict theta to [-pi pi]
  interp_state(2) = math::wrap_to_pi(interp_state(2));
  return interp_state;
}

//==============================================================================
double StateSpaceSE2::distance(
    const math::VectorXd& state1, const math::VectorXd& state2) const {
  CROBOT_ASSERT_ERROR(state1.size() == 3);
  CROBOT_ASSERT_ERROR(state2.size() == 3);

  // Find difference between states
  math::VectorXd state_diff = (state1 - state2).cwiseAbs();

  double dx     = state_diff(0);
  double dy     = state_diff(1);
  double dtheta = state_diff(2);

  // Restrict theta to [-pi pi]
  dtheta = math::wrap_to_pi(dtheta);

  // Distance for SE2 is implemented as Cartesian product of R2
  // and SO2.
  double dist = std::sqrt(
      m_weight_xy * (dx * dx + dy * dy) + m_weight_theta * (dtheta * dtheta));

  return dist;
}

//==============================================================================
bool StateSpaceSE2::is_state_valid(const math::VectorXd& state) const {
  CROBOT_ASSERT_ERROR(state.size() == 3);
  return (state.array() >= m_lb.array()).all()
         && (state.array() <= m_ub.array()).all();
}

//==============================================================================
bool StateSpaceSE2::is_transition_valid(
    const math::VectorXd& from, const math::VectorXd& to) const {
  if (!is_state_valid(from)) {
    return false;
  }
  if (!is_state_valid(to)) {
    return false;
  }
  return true;
}
