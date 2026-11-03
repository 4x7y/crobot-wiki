#pragma once

#include <memory>

#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rolling_grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class GraspXposCtrlTarget;
using GraspXposCtrlTargetPtr      = std::shared_ptr<GraspXposCtrlTarget>;
using ConstGraspXposCtrlTargetPtr = std::shared_ptr<const GraspXposCtrlTarget>;

class GraspXposCtrlTarget {
public:
  static GraspXposCtrlTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  GraspXposCtrlTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  virtual ~GraspXposCtrlTarget() = default;

  /**
   * @brief Compute the X ddot (object acceleration) in the body frame.
   *
   * @param qpos Current scene joint positions
   * @param qvel Current scene joint velocities
   * @param tforms Current scene transforms
   * @return <vec1 , vec2>
   *         vec1 : X ddot in the body frame;
   *         vec2 : wrench <force, torque> that needed to achieve the X ddot in
   * the target's body frame, with gravity compensation.
   */
  std::tuple<math::Vector6d, math::Vector6d> resolve(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const std::vector<math::Matrix4d>& tforms);

  kdl::ConstRigidBodyPtr get_body() const;

private:
  kdl::ConstFramePtr m_frame_desired;
  kdl::ConstRigidBodyPtr m_body;
  math::Matrix3d m_rotm;
  math::Vector3d m_bpos;

  int m_nv;

  math::MatrixXd KDv;
  math::MatrixXd KPv;
  math::MatrixXd KIv;

  math::Vector6d m_spos_err_sum;
};

} // namespace crobot::control

