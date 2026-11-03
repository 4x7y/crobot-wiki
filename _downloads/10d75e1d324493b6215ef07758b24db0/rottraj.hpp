#pragma once

#include "crobot/math/math.hpp"

namespace crobot::planning {

/**
 * @brief Generate trajectory between two orientations
 *
 * This function interpolates between two orientations R0 and RF, specified
 * as quaternion vectors.
 *
 * The function outputs the series of interpolated orientations in time
 * returned as an Mx1 quaternion object. The function also outputs a 3xM vector
 * of angular velocities (rad/s), OMEGA,and a 3xM vector of angular
 * accelerations (rad/s^2), ALPHA.
 */
void rottraj(
    const math::Quaternion& quat_s,
    const math::Quaternion& quat_f,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Quaternion>& quat_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj);

/**
 * @brief Generate trajectory between two orientations
 *
 * This function interpolates between two orientations R0 and RF, specified
 * as 3x3 rotation matrices.
 *
 * The function outputs the series of interpolated orientations in time
 * returned as a 3x3xM matrix of rotation matrices. The function also outputs a
 * 3xM vector of angular velocities (rad/s), OMEGA, and a 3xM vector of angular
 * accelerations (rad/s^2), ALPHA.
 */
void rottraj(
    const math::Matrix3d& rotm_s,
    const math::Matrix3d& rotm_f,
    const std::pair<double, double>& time_interval,
    const std::vector<double>& t,
    std::vector<math::Matrix3d>& rotm_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj);

/**
 * @brief Generate trajectory between two orientations
 *
 * This function interpolates between two orientations R0 and RF, specified
 * as quaternion vectors.
 *
 * The function outputs the series of interpolated orientations in time
 * returned as an Mx1 quaternion object. The function also outputs a 3xM vector
 * of angular velocities (rad/s), OMEGA,and a 3xM vector of angular
 * accelerations (rad/s^2), ALPHA.
 */
void rottraj(
    const math::Quaternion& quat_s,
    const math::Quaternion& quat_f,
    const std::vector<std::vector<double>>& time_scaling,
    std::vector<math::Quaternion>& quat_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj);

/**
 * @brief Generate trajectory between two orientations
 *
 * This function interpolates between two orientations R0 and RF, specified
 * as 3x3 rotation matrices.
 *
 * The function outputs the series of interpolated orientations in time
 * returned as a 3x3xM matrix of rotation matrices. The function also outputs a
 * 3xM vector of angular velocities (rad/s), OMEGA, and a 3xM vector of angular
 * accelerations (rad/s^2), ALPHA.
 */
void rottraj(
    const math::Matrix3d& rotm_s,
    const math::Matrix3d& rotm_f,
    const std::vector<std::vector<double>>& time_scaling,
    std::vector<math::Matrix3d>& rotm_traj,
    std::vector<math::Vector3d>& omega_traj,
    std::vector<math::Vector3d>& alpha_traj);

} // namespace crobot::planning

