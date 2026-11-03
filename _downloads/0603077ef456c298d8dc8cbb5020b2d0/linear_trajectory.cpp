#include "crobot/planning/linear_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
LinearTrajectory::LinearTrajectory(
    const math::VectorXd& qpos_start,
    const math::VectorXd& qpos_end,
    std::shared_ptr<const TimeScaling> time_scaling)
  : LinearTrajectory({qpos_start, qpos_end}, time_scaling) {
}

//==============================================================================
LinearTrajectory::LinearTrajectory(
    const std::vector<math::VectorXd>& qpos,
    std::shared_ptr<const TimeScaling> time_scaling)
  : Trajectory(), m_time_scaling(time_scaling) {
  m_qpos = qpos;
  CROBOT_ASSERT_ERROR(m_qpos.size() >= 2);
}

//==============================================================================
double LinearTrajectory::get_duration() const {
  return m_time_scaling->get_duration();
}

//==============================================================================
math::VectorXd LinearTrajectory::get_position(double time) const {
  double s          = m_time_scaling->get_scaling_factor(time);
  auto [idx1, idx2] = get_indices(time);
  return m_qpos[idx1] + (m_qpos[idx2] - m_qpos[idx1]) * s;
}

//==============================================================================
math::VectorXd LinearTrajectory::get_velocity(double time) const {
  double sd         = m_time_scaling->get_scaling_factor_deriv(time);
  auto [idx1, idx2] = get_indices(time);
  return (m_qpos[idx2] - m_qpos[idx1]) * sd;
}

//==============================================================================
math::VectorXd LinearTrajectory::get_acceleration(double time) const {
  double sd2        = m_time_scaling->get_scaling_factor_deriv2(time);
  auto [idx1, idx2] = get_indices(time);
  return (m_qpos[idx2] - m_qpos[idx1]) * sd2;
}

//==============================================================================
std::pair<std::size_t, std::size_t> LinearTrajectory::get_indices(
    double time) const {
  double duration = m_time_scaling->get_duration();
  if (time <= 0.0) {
    return std::make_pair<std::size_t, std::size_t>(0, 0);
  }
  else if (time >= duration) {
    return std::make_pair<std::size_t, std::size_t>(
        m_qpos.size() - 1, m_qpos.size() - 1);
  }
  else {
    std::size_t segments = m_qpos.size() - 1;
    double dt            = duration / static_cast<double>(segments);
    std::size_t idx1     = time / dt;
    std::size_t idx2     = idx1 + 1;
    return std::make_pair(idx1, idx2);
  }
}
