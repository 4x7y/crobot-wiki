#pragma once

#include <memory>

#include "crobot/control/task_space_target.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class TaskXfrcCtrlTarget;
using TaskXfrcCtrlTargetPtr      = std::shared_ptr<TaskXfrcCtrlTarget>;
using ConstTaskXfrcCtrlTargetPtr = std::shared_ptr<const TaskXfrcCtrlTarget>;

class TaskXfrcCtrlTarget : public TaskSpaceTarget {
public:
  static TaskXfrcCtrlTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      std::vector<int> vdir,
      std::vector<int> fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  TaskXfrcCtrlTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      std::vector<int> vdir,
      std::vector<int> fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  virtual ~TaskXfrcCtrlTarget() = default;

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
  math::MatrixXd m_SfC;
  int m_nctrl;
  int m_nf;
  int m_nv;

  math::MatrixXd KDv;
  math::MatrixXd KPv;
  math::MatrixXd KIv;

  double KPf;
  double KIf;

  math::VectorXd m_sfrc_desired;
  mutable math::VectorXd m_sfrc_error_int;
  mutable math::VectorXd m_spos_error_int;
  double m_dt;
};

} // namespace crobot::control

