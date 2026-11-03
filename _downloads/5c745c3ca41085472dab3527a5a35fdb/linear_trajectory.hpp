#pragma once

#include <memory>

#include "crobot/planning/time_scaling.hpp"
#include "crobot/planning/trajectory.hpp"

namespace crobot::planning {

class LinearTrajectory : public Trajectory {
public:
  using Ptr      = std::shared_ptr<LinearTrajectory>;
  using ConstPtr = std::shared_ptr<const LinearTrajectory>;

  LinearTrajectory(
      const math::VectorXd& qpos_start,
      const math::VectorXd& qpos_end,
      std::shared_ptr<const TimeScaling> time_scaling);

  LinearTrajectory(
      const std::vector<math::VectorXd>& qpos,
      std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  double get_duration() const override;

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

  math::VectorXd get_acceleration(double time) const;

private:
  std::pair<std::size_t, std::size_t> get_indices(double time) const;

  std::vector<math::VectorXd> m_qpos;
  std::shared_ptr<const TimeScaling> m_time_scaling;
};

} // namespace crobot::planning

