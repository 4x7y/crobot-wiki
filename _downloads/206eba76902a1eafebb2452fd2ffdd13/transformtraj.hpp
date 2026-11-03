#pragma once

#include "crobot/math/math.hpp"

namespace crobot::planning {

/**
 * @brief Generate trajectory between two homogeneous transforms.
 *
 * This function generates trajectories between two 4x4 homogeneous
 * transformation matrices T0 and TF, given a 1xM time vector T from zero to
 * one. The function outputs a 4x4xM matrix of transformations TFORM (position
 * elements in m), as well as a 6xM matrix VEL of the angular velocities (rad/s)
 * and velocities (m/s) in time, and a 6xM matrix ACC of the angular
 * accelerations (rad/s^2) and accelerations (m/s^2) in time.
 */
void transformtraj(
    const math::Transform& tform0,
    const math::Transform& tform1,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Transform>& tform_traj,
    std::vector<math::Vector6d>& vel_traj,
    std::vector<math::Vector6d>& acc_traj);

void transformtraj(
    const math::Matrix4d& tform0,
    const math::Matrix4d& tformf,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Matrix4d>& tform_traj,
    std::vector<math::Vector6d>& vel_traj,
    std::vector<math::Vector6d>& acc_traj);

} // namespace crobot::planning

