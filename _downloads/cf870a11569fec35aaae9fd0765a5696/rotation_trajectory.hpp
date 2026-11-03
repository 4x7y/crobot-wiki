#pragma once

#include "crobot/math/math.hpp"
#include "crobot/planning/time_scaling.hpp"
#include "crobot/planning/trajectory.hpp"

namespace crobot::planning {

class RotationTrajectory : public Trajectory {
public:
  RotationTrajectory(
      const math::Quaternion& quat_start,
      const math::Quaternion& quat_end,
      const std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  double get_duration() const override;

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

  math::VectorXd get_acceleration(double time) const;

  /**
   * @brief Get the orientation at a given time.
   *
   * @param s The time at which to get the orientation
   * @return math::Quaternion The orientation at the given time
   */
  math::Quaternion get_quat(double time) const;

  /**
   * @brief Get the angular velocity at a given time.
   *
   * @param[in] quat The orientation at the given time
   * @param[in] time The time at which to get the angular velocity
   * @return math::Vector3d The angular velocity at the given time
   */
  math::Vector3d get_omega(const math::Quaternion& quat, double time) const;

  /**
   * @brief Get the angular acceleration at a given time.
   *
   * @param[in] quat The orientation at the given time
   * @param[in] time The time at which to get the acceleration
   * @return math::Vector3d The angular acceleration at the given time
   */
  math::Vector3d get_alpha(const math::Quaternion& quat, double time) const;

  /**
   * @brief Get the orientation and angular velocity at a given time.
   *
   * @param time The time at which to get the orientation and angular velocity
   * @return math::Vector3d The orientation and angular velocity at the given
   * time
   */
  std::pair<math::Quaternion, math::Vector3d> get_quat_omega(double time) const;

private:
  math::Quaternion m_quat_start;
  math::Quaternion m_quat_end;
  std::shared_ptr<const TimeScaling> m_time_scaling;
};

} // namespace crobot::planning

