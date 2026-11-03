#pragma once

#include <memory>

#include "crobot/planning/linear_trajectory.hpp"
#include "crobot/planning/rotation_trajectory.hpp"
#include "crobot/planning/time_scaling.hpp"
#include "crobot/planning/trajectory.hpp"

namespace crobot::planning {

class ScrewdriverTrajectory : public Trajectory {
public:
  using Ptr      = std::shared_ptr<ScrewdriverTrajectory>;
  using ConstPtr = std::shared_ptr<const ScrewdriverTrajectory>;

  ScrewdriverTrajectory(
      const math::Vector3d& xpos_start,
      const math::Vector3d& xpos_end,
      const math::Quaternion& quat_start,
      const double angle_rotz,
      const std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  double get_duration() const override;

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

  math::VectorXd get_acceleration(double time) const;

  double get_angle_rotz(double time) const;

  /**
   * @brief Get the transformation at a given time.
   *
   * @param time Time at which to get the transformation.
   * @return math::Matrix4d The transformation at the given time.
   */
  math::Matrix4d get_tform(double time) const;

private:
  std::shared_ptr<LinearTrajectory> m_trans_traj;
  std::shared_ptr<LinearTrajectory> m_angle_rotz;
  std::shared_ptr<const TimeScaling> m_time_scaling;
  const math::Quaternion m_quat_start;
};

} // namespace crobot::planning

