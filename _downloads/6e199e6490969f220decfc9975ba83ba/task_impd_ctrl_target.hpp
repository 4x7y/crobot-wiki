#pragma once

#include <memory>

#include "crobot/control/task_space_target.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class TaskImpdCtrlTarget;
using TaskImpdCtrlTargetPtr         = std::shared_ptr<TaskImpdCtrlTarget>;
using ConstHybridImpedanceTargetPtr = std::shared_ptr<const TaskImpdCtrlTarget>;

class TaskImpdCtrlTarget : public TaskSpaceTarget {
public:
  static TaskImpdCtrlTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      std::vector<int> vdir,
      std::vector<int> fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  TaskImpdCtrlTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      std::vector<int> vdir,
      std::vector<int> fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  virtual ~TaskImpdCtrlTarget() = default;

  void set_desired_force(const math::VectorXd& sfrc);

  std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd> resolve(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const std::vector<math::Matrix4d>& tform) const override;

private:
  kdl::ConstFramePtr m_frame_desired;
  kdl::ConstRigidBodyPtr m_body;
  math::Matrix3d m_rotm;
  math::Vector3d m_bpos;

  kdl::CollisionContact::ConstPtr m_contact;

  math::MatrixXd m_S;
  math::MatrixXd m_Sv;
  math::MatrixXd m_Sf;
  int m_nctrl;
  int m_nf;
  int m_nv;

  math::MatrixXd KDv;
  math::MatrixXd KPv;

  math::MatrixXd KPm;

  math::MatrixXd KMi;
  math::MatrixXd KDi;
  math::MatrixXd KPi;

  math::VectorXd m_sfrc_desired;
};

} // namespace crobot::control

