#pragma once

#include <memory>

#include "crobot/control/grasp_xpos_ctrl_target.hpp"
#include "crobot/control/task_space_target.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/kdl/rolling_grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

// forward declaration
class GraspController;
using GraspControllerPtr      = std::shared_ptr<GraspController>;
using ConstGraspControllerPtr = std::shared_ptr<const GraspController>;

class GraspController {
public:
  static GraspControllerPtr MakeShared(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::RollingGrasp::Ptr grasp,
      math::Vector3d& frc_norm);

  GraspController(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::RollingGrasp::Ptr grasp,
      math::Vector3d& frc_norm);

  math::VectorXd compute(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const std::vector<math::Vector3d>& frc_norm) const;

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
  GraspXposCtrlTargetPtr add_grasp_xpos_target(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::string& ctrl_param_file);

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  GraspXposCtrlTargetPtr m_target;
  kdl::RollingGrasp::Ptr m_grasp;
  math::Vector3d m_frc_norm;

  struct
  {
    math::VectorXd qpos_lb;
    math::VectorXd qpos_ub;
    math::VectorXd qpos_center;
  } m_null_space_obj;
};

} // namespace crobot::control

