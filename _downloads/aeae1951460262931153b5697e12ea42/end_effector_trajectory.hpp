#pragma once

#include "crobot/planning/body_trajectory.hpp"
#include "crobot/planning/pose_trajectory.hpp"
#include "crobot/planning/time_scaling.hpp"

namespace crobot::planning {

class EndEffectorTrajectory : public BodyTrajectory {

  EndEffectorTrajectory(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name,
      const math::VectorXd& qpos_start,
      const math::Matrix4d& tform_end,
      std::shared_ptr<const TimeScaling> time_scaling);

  // Documentation inherited
  using BodyTrajectory::get_duration;

  // Documentation inherited
  math::VectorXd get_position(double time) const override;

  // Documentation inherited
  math::VectorXd get_velocity(double time) const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;
  math::VectorXd m_qpos_start;
  std::shared_ptr<PoseTrajectory> m_pose_traj;
  std::shared_ptr<const TimeScaling> m_time_scaling;
  std::vector<int> m_qvel_addr;
};

} // namespace crobot::planning

