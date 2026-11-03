#include "crobot/planning/pivot_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
PivotTrajectory::PivotTrajectory(
    PivotTarget::ConstPtr target,
    const math::Quaternion& quat_start,
    const math::Quaternion& quat_end,
    std::shared_ptr<const TimeScaling> time_scaling)
  : BodyTrajectory(target->get_body(), time_scaling->get_duration()),
    m_quat_start(quat_start),
    m_quat_end(quat_end),
    m_bpos_pivot(target->get_pivot_bpos()),
    m_xpos_pivot(target->get_pivot_xpos()) {
  m_rot_traj = std::make_shared<RotationTrajectory>(
      quat_start, quat_end, time_scaling);
}

//==============================================================================
math::VectorXd PivotTrajectory::get_position(double time) const {
  math::Quaternion xquat = m_rot_traj->get_quat(time);
  math::Vector3d xpos    = m_xpos_pivot - xquat * m_bpos_pivot;
  math::VectorXd qpos(7);
  qpos << xpos, xquat.as_vector();
  return qpos;
}

//==============================================================================
math::VectorXd PivotTrajectory::get_velocity(double time) const {
  math::Quaternion xquat;
  math::Vector3d omega;
  std::tie(xquat, omega) = m_rot_traj->get_quat_omega(time);
  math::VectorXd qvel(6);
  qvel << (xquat * m_bpos_pivot).cross(omega), omega;
  return qvel;
}

//==============================================================================
math::VectorXd PivotTrajectory::get_acceleration(double time) const {
  math::Quaternion xquat;
  math::Vector3d omega;
  std::tie(xquat, omega) = m_rot_traj->get_quat_omega(time);
  math::Vector3d alpha   = m_rot_traj->get_alpha(xquat, time);
  math::VectorXd qacc(6);
  math::Vector3d xacc = (xquat * m_bpos_pivot).cross(alpha)
                        - omega.cross(omega.cross(xquat * m_bpos_pivot));
  qacc << xacc, omega;
  return qacc;
}
