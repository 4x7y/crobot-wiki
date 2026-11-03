#include "crobot/planning/rottraj.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/planning/time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
void crobot::planning::rottraj(
    const math::Quaternion& quat_s,
    const math::Quaternion& quat_f,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Quaternion>& quat_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj) {
  auto time_scaling = construct_linear_time_scaling(time_interval, t);
  rottraj(quat_s, quat_f, time_scaling, quat_traj, omega_traj, alpha_traj);
}

//==============================================================================
void crobot::planning::rottraj(
    const math::Matrix3d& rotm_s,
    const math::Matrix3d& rotm_f,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Matrix3d>& rotm_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj) {
  auto time_scaling = construct_linear_time_scaling(time_interval, t);
  rottraj(rotm_s, rotm_f, time_scaling, rotm_traj, omega_traj, alpha_traj);
}

//==============================================================================
void crobot::planning::rottraj(
    const math::Matrix3d& rotm_s,
    const math::Matrix3d& rotm_f,
    const std::vector<std::vector<double>>& time_scaling,
    std::vector<math::Matrix3d>& rotm_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj) {

  CROBOT_ASSERT_ERROR(time_scaling.size() == 3);
  CROBOT_ASSERT_ERROR(time_scaling[0].size() == time_scaling[1].size());
  CROBOT_ASSERT_ERROR(time_scaling[0].size() == time_scaling[2].size());

  std::vector<math::Quaternion> quat_traj;
  rottraj(
      math::Quaternion(rotm_s),
      math::Quaternion(rotm_f),
      time_scaling,
      quat_traj,
      omega_traj,
      alpha_traj);
  rotm_traj.reserve(quat_traj.size());
  for (auto& q : quat_traj) {
    rotm_traj.push_back(q.as_rotm());
  }
}

//==============================================================================
void crobot::planning::rottraj(
    const math::Quaternion& quat_s,
    const math::Quaternion& quat_f,
    const std::vector<std::vector<double>>& time_scaling,
    std::vector<math::Quaternion>& quat_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj) {

  CROBOT_ASSERT_ERROR(time_scaling.size() == 3);
  CROBOT_ASSERT_ERROR(time_scaling[0].size() == time_scaling[1].size());
  CROBOT_ASSERT_ERROR(time_scaling[0].size() == time_scaling[2].size());

  auto s   = time_scaling[0];
  auto sd  = time_scaling[1];
  auto sdd = time_scaling[2];

  // initialize outputs
  std::size_t m = s.size();
  quat_traj.clear();
  quat_traj.reserve(m);
  omega_traj.clear();
  omega_traj.reserve(m);
  alpha_traj.clear();
  alpha_traj.reserve(m);

  // prepare inputs
  math::Quaternion pn = quat_s;
  pn.normalize();
  math::Quaternion qn = quat_f;
  qn.normalize();

  // get corrected start and end values from quaternion slerp method
  math::Quaternion pn_corrected = pn.slerp(0.0, qn);
  math::Quaternion qn_corrected = pn.slerp(1.0, qn);

  for (size_t i = 0; i < m; i++) {
    quat_traj.push_back(pn.slerp(s[i], qn));
    // Compute angular velocity and acceleration from the quaternion
    // derivatives : omega = 2(dq / dt)(q*), alpha = 2(d ^ 2q / dt ^ 2)(q*)
    math::Quaternion qdCalc = pn_corrected.slerp_first_derivative(
        sd[i], qn_corrected, quat_traj[i]);
    math::Vector3d W = (qdCalc * quat_traj[i].conjugate() * 2.0).vec();
    omega_traj.push_back(W);

    math::Quaternion qddCalc = pn_corrected.slerp_second_derivative(
        sdd[i], qn_corrected, quat_traj[i]);
    math::Vector3d A = (qddCalc * quat_traj[i].conjugate() * 2.0).vec();
    alpha_traj.push_back(A);
  }
}
