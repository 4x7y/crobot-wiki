#pragma once

#include <memory>

#include "crobot/control/task_space_target.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class TaskXposCtrlTarget;
using TaskXposCtrlTargetPtr      = std::shared_ptr<TaskXposCtrlTarget>;
using ConstTaskXposCtrlTargetPtr = std::shared_ptr<const TaskXposCtrlTarget>;

class TaskXposCtrlTarget : public TaskSpaceTarget {
public:
  static TaskXposCtrlTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  TaskXposCtrlTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  virtual ~TaskXposCtrlTarget() = default;

  std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd> resolve(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const std::vector<math::Matrix4d>& tforms) const override;

private:
  kdl::ConstFramePtr m_frame_desired;
  kdl::ConstRigidBodyPtr m_body;
  math::Matrix3d m_rotm;
  math::Vector3d m_bpos;
  kdl::CollisionContact::ConstPtr m_contact;

  math::Matrix3d KDo;
  math::Matrix3d KPo;
  math::Matrix3d KDp;
  math::Matrix3d KPp;
};

} // namespace crobot::control

