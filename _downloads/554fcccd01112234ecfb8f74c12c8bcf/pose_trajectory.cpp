#include "crobot/planning/pose_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
PoseTrajectory::PoseTrajectory(
    const math::Matrix4d& tform_start,
    const math::Matrix4d& tform_end,
    const std::shared_ptr<const TimeScaling> time_scaling)
  : Trajectory(),
    m_trans_traj(std::make_shared<LinearTrajectory>(
        tform_start.tform2trvec(), tform_end.tform2trvec(), time_scaling)),
    m_rotm_traj(std::make_shared<RotationTrajectory>(
        tform_start.tform2quat(), tform_end.tform2quat(), time_scaling)),
    m_time_scaling(time_scaling) {
}

//==============================================================================
double PoseTrajectory::get_duration() const {
  return m_time_scaling->get_duration();
}

//==============================================================================
math::VectorXd PoseTrajectory::get_position(double time) const {
  math::Vector4d quat = m_rotm_traj->get_position(time);
  math::Vector3d xpos = m_trans_traj->get_position(time);
  math::VectorXd qpos(7);
  qpos << xpos, quat;
  return qpos;
}

//==============================================================================
math::VectorXd PoseTrajectory::get_velocity(double time) const {
  math::Vector3d xvel = m_trans_traj->get_velocity(time);
  math::Vector3d omeg = m_rotm_traj->get_velocity(time);
  math::VectorXd qvel(6);
  qvel << omeg, xvel;
  return qvel;
}

//==============================================================================
math::VectorXd PoseTrajectory::get_acceleration(double time) const {
  math::Vector3d xacc  = m_trans_traj->get_acceleration(time);
  math::Vector3d alpha = m_rotm_traj->get_acceleration(time);
  math::VectorXd qacc(6);
  qacc << alpha, xacc;
  return qacc;
}

//==============================================================================
math::Matrix4d PoseTrajectory::get_tform(double time) const {
  math::VectorXd qpos   = get_position(time);
  math::Quaternion quat = math::Quaternion(qpos[3], qpos[4], qpos[5], qpos[6]);
  math::Matrix4d tform  = quat.as_tform();
  tform.block<3, 1>(0, 3) = qpos.head<3>();
  return tform;
}
