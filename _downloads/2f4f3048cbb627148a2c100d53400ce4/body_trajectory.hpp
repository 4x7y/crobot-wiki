#pragma once

#include "crobot/kdl/rigid_body.hpp"
#include "crobot/planning/trajectory.hpp"

namespace crobot::planning {

class BodyTrajectory : public Trajectory {
public:
  BodyTrajectory(kdl::ConstRigidBodyPtr body, double duration);

  double get_duration() const override;

  kdl::ConstRigidBodyPtr get_body() const;

private:
  kdl::ConstRigidBodyPtr m_body;
  double m_duration;
};

} // namespace crobot::planning

