#pragma once

#include <memory>

#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rolling_grasp.hpp"
#include "crobot/kdl/site.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class ObjXposCtrlTarget;
using ObjXposCtrlTargetPtr      = std::shared_ptr<ObjXposCtrlTarget>;
using ConstObjXposCtrlTargetPtr = std::shared_ptr<const ObjXposCtrlTarget>;

class ObjXposCtrlTarget {
public:
  static ObjXposCtrlTargetPtr MakeShared(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  ObjXposCtrlTarget(
      kdl::ConstFramePtr frame_desired,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

  virtual ~ObjXposCtrlTarget() = default;

  /**
   * @brief Compute the X ddot (object acceleration) in the body frame.
   *
   * @param tforms The forward kinematics of the robot
   * @return math::Matrix4d The desired xform in the next time step
   */
  math::Matrix4d resolve(
      const math::Matrix4d& xform_actual, const math::Matrix4d& xform_cmd);

  kdl::ConstRigidBodyPtr get_body() const;

private:
  kdl::ConstFramePtr m_frame_desired;
  kdl::ConstRigidBodyPtr m_body;
  math::Matrix3d m_rotm;
  math::Vector3d m_bpos;

  int m_nv;
  double m_dt;
  double m_decay;
  double m_decay_factor;

  math::MatrixXd KDv;
  math::MatrixXd KPv;
  math::MatrixXd KIv;

  math::Matrix4d m_tform_ctrl;
  math::Matrix4d m_xform_last_fb;
};

} // namespace crobot::control

