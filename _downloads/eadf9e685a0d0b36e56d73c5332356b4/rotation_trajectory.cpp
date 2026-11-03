#include "crobot/planning/rotation_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RotationTrajectory::RotationTrajectory(
    const math::Quaternion& quat_start,
    const math::Quaternion& quat_end,
    const std::shared_ptr<const TimeScaling> time_scaling)
  : Trajectory(), m_time_scaling(time_scaling) {
  // prepare inputs
  math::Quaternion pn = quat_start;
  pn.normalize();
  math::Quaternion qn = quat_end;
  qn.normalize();

  // get corrected start and end values from quaternion slerp method
  m_quat_start = pn.slerp(0.0, qn);
  m_quat_end   = pn.slerp(1.0, qn);
}

//==============================================================================
double RotationTrajectory::get_duration() const {
  return m_time_scaling->get_duration();
}

//==============================================================================
math::VectorXd RotationTrajectory::get_position(double time) const {
  return get_quat(time).as_vector();
}

//==============================================================================
math::VectorXd RotationTrajectory::get_velocity(double time) const {
  return get_omega(get_quat(time), time);
}

//==============================================================================
math::VectorXd RotationTrajectory::get_acceleration(double time) const {
  return get_alpha(get_quat(time), time);
}

//==============================================================================
math::Quaternion RotationTrajectory::get_quat(double time) const {
  return m_quat_start.slerp(
      m_time_scaling->get_scaling_factor(time), m_quat_end);
}

//==============================================================================
math::Vector3d RotationTrajectory::get_omega(
    const math::Quaternion& quat, double time) const {
  math::Quaternion qdCalc = m_quat_start.slerp_first_derivative(
      m_time_scaling->get_scaling_factor_deriv(time), m_quat_end, quat);
  math::Vector3d omega = (qdCalc * quat.conjugate() * 2.0).vec();
  return omega;
}

//==============================================================================
math::Vector3d RotationTrajectory::get_alpha(
    const math::Quaternion& quat, double time) const {
  math::Quaternion qddCalc = m_quat_start.slerp_second_derivative(
      m_time_scaling->get_scaling_factor_deriv(time), m_quat_end, quat);
  math::Vector3d alpha = (qddCalc * quat.conjugate() * 2.0).vec();
  return alpha;
}

//==============================================================================
std::pair<math::Quaternion, math::Vector3d> RotationTrajectory::get_quat_omega(
    double time) const {
  math::Quaternion quat = get_quat(time);
  math::Vector3d omega  = get_omega(quat, time);
  return std::make_pair(quat, omega);
}
