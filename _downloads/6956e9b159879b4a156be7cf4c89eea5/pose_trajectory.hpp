#pragma once

#include <memory>

#include "crobot/planning/linear_trajectory.hpp"
#include "crobot/planning/rotation_trajectory.hpp"
#include "crobot/planning/time_scaling.hpp"
#include "crobot/planning/trajectory.hpp"

namespace crobot::planning {

class PoseTrajectory : public Trajectory {
public:
  using Ptr      = std::shared_ptr<PoseTrajectory>;
  using ConstPtr = std::shared_ptr<const PoseTrajectory>;

  PoseTrajectory(
      const math::Matrix4d& tform_start,
      const math::Matrix4d& tform_end,
      const std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  double get_duration() const override;

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

  math::VectorXd get_acceleration(double time) const;

  /**
   * @brief Get the transformation at a given time.
   *
   * @param time Time at which to get the transformation.
   * @return math::Matrix4d The transformation at the given time.
   */
  math::Matrix4d get_tform(double time) const;

private:
  std::shared_ptr<LinearTrajectory> m_trans_traj;
  std::shared_ptr<RotationTrajectory> m_rotm_traj;
  std::shared_ptr<const TimeScaling> m_time_scaling;
};

} // namespace crobot::planning

