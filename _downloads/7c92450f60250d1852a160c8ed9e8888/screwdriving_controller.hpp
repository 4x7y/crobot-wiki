#pragma once

#include <memory>

#include "crobot/control/screwdriving_ctrl_target.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/kdl/rolling_grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

// forward declaration
class ScrewdrivingController;
using ScrewdrivingControllerPtr = std::shared_ptr<ScrewdrivingController>;
using ConstScrewdrivingControllerPtr
    = std::shared_ptr<const ScrewdrivingController>;

class ScrewdrivingController {
public:
  static ScrewdrivingControllerPtr MakeShared(
      kdl::ConstRigidBodyTreePtr rbtree, kdl::RollingGrasp::Ptr grasp);

  ScrewdrivingController(
      kdl::ConstRigidBodyTreePtr rbtree, kdl::RollingGrasp::Ptr grasp);

  math::VectorXd compute(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::Vector6d& fext_desired,
      const std::vector<math::Vector3d>&) const;

  /**
   * @brief Add a task space position target to the controller.
   *
   * @param desired_frame Target frame in the world frame.
   * @param body RigidBody of the robot that corresponds to the target frame
   * motion.
   * @param rotm Local frame rotation w.r.t. the body's body frame
   * @param bpos Local frame position w.r.t. the body's body frame
   * @param contact Contact point for force/torque reading.
   */
  ScrewdrivingCtrlTargetPtr add_screwdriving_ctrl_target(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  ScrewdrivingCtrlTargetPtr m_target;
  kdl::RollingGrasp::Ptr m_grasp;

  struct
  {
    math::VectorXd qpos_lb;
    math::VectorXd qpos_ub;
    math::VectorXd qpos_center;
  } m_null_space_obj;
};

} // namespace crobot::control

