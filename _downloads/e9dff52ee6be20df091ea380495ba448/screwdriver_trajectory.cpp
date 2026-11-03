#include "crobot/planning/screwdriver_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
ScrewdriverTrajectory::ScrewdriverTrajectory(
    const math::Vector3d& xpos_start,
    const math::Vector3d& xpos_end,
    const math::Quaternion& quat_start,
    const double angle_rotz,
    const std::shared_ptr<const TimeScaling> time_scaling)
  : Trajectory(),
    m_trans_traj(
        std::make_shared<LinearTrajectory>(xpos_start, xpos_end, time_scaling)),
    m_angle_rotz(std::make_shared<LinearTrajectory>(
        math::Vector1d(0.0), math::Vector1d(angle_rotz), time_scaling)),
    m_time_scaling(time_scaling),
    m_quat_start(quat_start) {
}

//==============================================================================
double ScrewdriverTrajectory::get_duration() const {
  return m_time_scaling->get_duration();
}

//==============================================================================
math::VectorXd ScrewdriverTrajectory::get_position(double time) const {
  double angle = m_angle_rotz->get_position(time)[0];
  math::Vector4d quat
      = (m_quat_start * math::AngleAxisd(angle, math::Vector3d::UnitZ()))
            .as_vector();

  math::Vector3d xpos = m_trans_traj->get_position(time);
  math::VectorXd qpos(7);
  qpos << xpos, quat;
  return qpos;
}

//==============================================================================
math::VectorXd ScrewdriverTrajectory::get_velocity(double time) const {
  math::Vector3d xvel = m_trans_traj->get_velocity(time);
  math::Vector3d bomeg
      = m_angle_rotz->get_velocity(time)[0] * math::Vector3d::UnitZ();
  math::Vector3d omeg = m_quat_start * bomeg;
  math::VectorXd qvel(6);
  // qvel << xvel, omeg;
  qvel << omeg, xvel;
  return qvel;
}

//==============================================================================
math::VectorXd ScrewdriverTrajectory::get_acceleration(double time) const {
  math::Vector3d xacc = m_trans_traj->get_acceleration(time);
  math::Vector3d balpha
      = m_angle_rotz->get_acceleration(time)[0] * math::Vector3d::UnitZ();
  math::Vector3d alpha = m_quat_start * balpha;
  math::VectorXd qacc(6);
  // qacc << xacc, alpha;
  qacc << alpha, xacc;
  return qacc;
}

//==============================================================================
double ScrewdriverTrajectory::get_angle_rotz(double time) const {
  return m_angle_rotz->get_position(time)[0];
}

//==============================================================================
math::Matrix4d ScrewdriverTrajectory::get_tform(double time) const {
  math::VectorXd xpos_quat = get_position(time);

  math::Vector4d quat_vec;
  quat_vec << xpos_quat.tail<3>(), xpos_quat[3];

  math::Quaternion quat   = math::Quaternion(quat_vec);
  math::Vector3d xpos     = xpos_quat.head<3>();
  math::Matrix4d tform    = quat.as_tform();
  tform.block<3, 1>(0, 3) = xpos;
  return tform;
}
