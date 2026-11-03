#pragma once

#include <memory>

#include "crobot/control/task_impd_ctrl_target.hpp"
#include "crobot/control/task_space_target.hpp"
#include "crobot/control/task_xfrc_ctrl_target.hpp"
#include "crobot/control/task_xpos_ctrl_target.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
namespace crobot::control {

// forward declaration
class TaskSpaceController;
using TaskSpaceControllerPtr      = std::shared_ptr<TaskSpaceController>;
using ConstTaskSpaceControllerPtr = std::shared_ptr<const TaskSpaceController>;

class TaskSpaceController {
public:
  static TaskSpaceControllerPtr MakeShared(kdl::ConstRigidBodyTreePtr rbtree);

  TaskSpaceController(kdl::ConstRigidBodyTreePtr rbtree);

  math::VectorXd compute(
      const math::VectorXd& qpos, const math::VectorXd& qvel);

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
  TaskXposCtrlTargetPtr add_xpos_target(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  /**
   * @brief Add a task space hybrid position/impedance control target to the
   * controller. The specified coordinates of the local frame (represented by
   * rotm, bpos, and body) in the target frame will become 0 if the controller
   * converges.
   *
   * @param desired_frame Target frame in the world frame.
   * @param body RigidBody of the robot that corresponds to the target frame
   * motion.
   * @param rotm Local frame rotation w.r.t. the body's body frame
   * @param bpos Local frame position w.r.t. the body's body frame
   * @param vdir Indices for coordinates under position control.
   * @param fdir Indices for coordinates under impedance control.
   * @param contact Contact point for force/torque reading.
   */
  TaskImpdCtrlTargetPtr add_hyrbid_impd_target(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::vector<int>& vdir,
      const std::vector<int>& fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

  /**
   * @brief Add a task space hybrid position/force control target to the
   * controller. The specified coordinates of the local frame (represented by
   * rotm, bpos, and body) in the target frame will become 0 if the controller
   * converges.
   *
   * @param desired_frame Target frame in the world frame.
   * @param body RigidBody of the robot that corresponds to the target frame
   * motion.
   * @param rotm Local frame rotation w.r.t. the body's body frame
   * @param bpos Local frame position w.r.t. the body's body frame
   * @param vdir Indices for coordinates under position control.
   * @param fdir Indices for coordinates under impedance control.
   * @param contact Contact point for force/torque reading.
   */
  TaskXfrcCtrlTargetPtr add_hyrbid_force_target(
      kdl::ConstFramePtr desired_frame,
      kdl::ConstRigidBodyPtr body,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos,
      const std::vector<int>& vdir,
      const std::vector<int>& fdir,
      kdl::CollisionContact::ConstPtr contact = nullptr);

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  std::vector<TaskSpaceTargetPtr> m_targets;
  int m_ndof;
  int m_nqvel;
  int m_nctrl;

  math::MatrixXd m_Q;
  math::MatrixXd m_U;
};

} // namespace crobot::control

