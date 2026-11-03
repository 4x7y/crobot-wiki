#pragma once

#include "crobot/planning/body_trajectory.hpp"
#include "crobot/planning/pivot_target.hpp"
#include "crobot/planning/rotation_trajectory.hpp"
#include "crobot/planning/time_scaling.hpp"

namespace crobot::planning {

class PivotTrajectory : public BodyTrajectory {
public:
  using Ptr      = std::shared_ptr<PivotTrajectory>;
  using ConstPtr = std::shared_ptr<const PivotTrajectory>;

  PivotTrajectory(
      PivotTarget::ConstPtr body,
      const math::Quaternion& quat_start,
      const math::Quaternion& quat_end,
      std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

  math::VectorXd get_acceleration(double time) const;

private:
  const math::Quaternion m_quat_start;
  const math::Quaternion m_quat_end;
  const math::Vector3d m_bpos_pivot;
  const math::Vector3d m_xpos_pivot;

  std::shared_ptr<RotationTrajectory> m_rot_traj;
};

} // namespace crobot::planning

