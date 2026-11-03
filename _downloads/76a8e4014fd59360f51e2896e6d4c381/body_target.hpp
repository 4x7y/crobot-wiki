#pragma once

#include <memory>

#include "crobot/control/single_target.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rolling_grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class BodyTarget;
using BodyTargetPtr      = std::shared_ptr<BodyTarget>;
using ConstBodyTargetPtr = std::shared_ptr<const BodyTarget>;

class BodyTarget {
public:
  static BodyTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  BodyTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  virtual ~BodyTarget() = default;

  /**
   * @brief Compute the X ddot (object acceleration) in the body frame.
   *
   * @param qpos Current scene joint positions
   * @param qvel Current scene joint velocities
   * @param fext Current scene external wrenches applied to the body
   * @param fext_desired Desired scene external wrenches applied to the body
   * @param tforms Current scene transforms
   * @return <vec1 , vec2>
   *         vec1 : X ddot in the body frame;
   *         vec2 : wrench <force, torque> that needed to achieve the X ddot in
   * the target's body frame, with gravity compensation.
   */
  std::tuple<math::Vector6d, math::Vector6d> resolve(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::Vector6d& xwrench,
      const std::vector<math::Matrix4d>& tforms) const;

  kdl::ConstRigidBodyPtr get_body() const;

private:
  kdl::ConstFramePtr m_frame_desired;
  kdl::ConstRigidBodyPtr m_body;
  math::Matrix3d m_rotm;
  math::Vector3d m_bpos;
  double m_dt;

  std::vector<std::shared_ptr<SingleTarget>> m_targets;
};

} // namespace crobot::control

