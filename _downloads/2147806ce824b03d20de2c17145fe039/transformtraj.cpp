#include "crobot/planning/transformtraj.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/planning/rottraj.hpp"
#include "crobot/planning/time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
void crobot::planning::transformtraj(
    const math::Transform& tform0,
    const math::Transform& tformf,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Transform>& tform_traj,
    std::vector<math::Vector6d>& vel_traj,
    std::vector<math::Vector6d>& acc_traj) {

  auto time_scaling = construct_linear_time_scaling(time_interval, t);
  auto s            = time_scaling[0];
  auto sd           = time_scaling[1];
  auto sdd          = time_scaling[2];

  math::Matrix3d rotm0 = tform0.rotation();
  math::Matrix3d rotmf = tformf.rotation();

  std::vector<math::Matrix3d> rotm_traj;
  std::vector<math::Vector3d> omega_traj;
  std::vector<math::Vector3d> alpha_traj;

  rottraj(rotm0, rotmf, time_scaling, rotm_traj, omega_traj, alpha_traj);

  std::size_t m = s.size();
  tform_traj.clear();
  tform_traj.reserve(m);
  vel_traj.clear();
  vel_traj.reserve(m);
  acc_traj.clear();
  acc_traj.reserve(m);

  math::Vector3d p0 = tform0.translation();
  math::Vector3d pf = tformf.translation();
  for (std::size_t i = 0; i < m; ++i) {
    math::Vector3d p   = p0 + (pf - p0) * s[i];
    math::Vector3d pd  = (pf - p0) * sd[i];
    math::Vector3d pdd = (pf - p0) * sdd[i];
    math::Vector3d w   = omega_traj[i];
    math::Vector3d a   = alpha_traj[i];

    math::Matrix4d tform    = math::Matrix4d::Identity();
    tform.block<3, 3>(0, 0) = rotm_traj[i];
    tform.block<3, 1>(0, 3) = p;
    tform_traj.push_back(math::Transform(tform));

    vel_traj.push_back(math::compose(w, pd));
    acc_traj.push_back(math::compose(a, pdd));
  }
}

//==============================================================================
void crobot::planning::transformtraj(
    const math::Matrix4d& tform0,
    const math::Matrix4d& tformf,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Matrix4d>& tform_traj,
    std::vector<math::Vector6d>& vel_traj,
    std::vector<math::Vector6d>& acc_traj) {

  auto time_scaling = construct_linear_time_scaling(time_interval, t);
  auto s            = time_scaling[0];
  auto sd           = time_scaling[1];
  auto sdd          = time_scaling[2];

  math::Matrix3d rotm0 = tform0.block<3, 3>(0, 0);
  math::Matrix3d rotmf = tformf.block<3, 3>(0, 0);

  std::vector<math::Matrix3d> rotm_traj;
  std::vector<math::Vector3d> omega_traj;
  std::vector<math::Vector3d> alpha_traj;

  rottraj(rotm0, rotmf, time_scaling, rotm_traj, omega_traj, alpha_traj);

  std::size_t m = s.size();
  tform_traj.clear();
  tform_traj.reserve(m);
  vel_traj.clear();
  vel_traj.reserve(m);
  acc_traj.clear();
  acc_traj.reserve(m);

  math::Vector3d p0 = tform0.block<3, 1>(0, 3);
  math::Vector3d pf = tformf.block<3, 1>(0, 3);
  for (std::size_t i = 0; i < m; ++i) {
    math::Vector3d p   = p0 + (pf - p0) * s[i];
    math::Vector3d pd  = (pf - p0) * sd[i];
    math::Vector3d pdd = (pf - p0) * sdd[i];
    math::Vector3d w   = omega_traj[i];
    math::Vector3d a   = alpha_traj[i];

    math::Matrix4d tform    = math::Matrix4d::Identity();
    tform.block<3, 3>(0, 0) = rotm_traj[i];
    tform.block<3, 1>(0, 3) = p;
    tform_traj.push_back(tform);

    vel_traj.push_back(math::compose(w, pd));
    acc_traj.push_back(math::compose(a, pdd));
  }
}
