#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/control/task_space_controller.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;
using namespace math;

//==============================================================================
TaskSpaceControllerPtr TaskSpaceController::MakeShared(
    kdl::ConstRigidBodyTreePtr rbtree) {
  return std::make_shared<TaskSpaceController>(std::move(rbtree));
}

//==============================================================================
TaskSpaceController::TaskSpaceController(ConstRigidBodyTreePtr rbtree)
  : m_rbtree(std::move(rbtree)) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  m_Q = m_rbtree->get_tmat_qvel2uvel(m_rbtree->get_home_qpos()).transpose();
  m_U = m_rbtree->get_tmat_uvel2qvel(m_rbtree->get_home_qpos());
  CROBOT_LOG_DEBUG("U = R^({}x{})", m_U.rows(), m_U.cols());
  CROBOT_LOG_DEBUG("Q = R^({}x{})", m_Q.rows(), m_Q.cols());
  CROBOT_LOG_DEBUG("U = \n{}", m_U);
  CROBOT_LOG_DEBUG("Q = \n{}", m_Q);

  m_nqvel = m_rbtree->get_qvel_dim();
  m_nctrl = 0.0;
}

//==============================================================================
math::VectorXd TaskSpaceController::compute(
    const math::VectorXd& qpos, const math::VectorXd& qvel) {
  m_nctrl = m_rbtree->get_ctrl_dim();

  math::MatrixXd J                   = math::MatrixXd::Zero(m_nctrl, m_nqvel);
  math::VectorXd sacc_resolved       = math::VectorXd::Zero(m_nctrl);
  math::VectorXd qfrc_resolved       = math::VectorXd::Zero(m_nqvel);
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  int row = 0;
  for (TaskSpaceTargetPtr target : m_targets) {
    math::VectorXd sacc_resolved_i;
    math::MatrixXd Ji;
    math::VectorXd qfrc_resolved_i;
    std::tie(sacc_resolved_i, qfrc_resolved_i, Ji)
        = target->resolve(qpos, qvel, tforms);

    J.middleRows(row, Ji.rows())          = Ji;
    sacc_resolved.segment(row, Ji.rows()) = sacc_resolved_i;
    qfrc_resolved += qfrc_resolved_i;
    row += Ji.rows();
  }
  CROBOT_ASSERT_ERROR(row == m_nctrl, "row=%d, nctrl=%d", row, m_nctrl);

  math::VectorXd qacc_resolved
      = m_Q * (J * m_Q).completeOrthogonalDecomposition().pseudoInverse()
        * sacc_resolved;

  qfrc_resolved += m_rbtree->inverse_dynamics(qpos, qvel, qacc_resolved);
  math::VectorXd ctrl = m_U.transpose() * qfrc_resolved;
  return ctrl;
}

//==============================================================================
TaskXposCtrlTargetPtr TaskSpaceController::add_xpos_target(
    ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    kdl::CollisionContact::ConstPtr contact) {
  TaskXposCtrlTargetPtr target = TaskXposCtrlTarget::MakeShared(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      std::move(contact));
  m_targets.push_back(target);
  return target;
}

//==============================================================================
TaskImpdCtrlTargetPtr TaskSpaceController::add_hyrbid_impd_target(
    ConstFramePtr frame_desired,
    ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::vector<int>& vdir,
    const std::vector<int>& fdir,
    kdl::CollisionContact::ConstPtr contact) {
  TaskImpdCtrlTargetPtr target = TaskImpdCtrlTarget::MakeShared(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      vdir,
      fdir,
      std::move(contact));
  m_targets.push_back(target);
  m_nctrl += vdir.size();
  return target;
}

//==============================================================================
TaskXfrcCtrlTargetPtr TaskSpaceController::add_hyrbid_force_target(
    ConstFramePtr frame_desired,
    ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::vector<int>& vdir,
    const std::vector<int>& fdir,
    kdl::CollisionContact::ConstPtr contact) {
  TaskXfrcCtrlTargetPtr target = TaskXfrcCtrlTarget::MakeShared(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      vdir,
      fdir,
      std::move(contact));
  m_targets.push_back(target);
  m_nctrl += vdir.size();
  return target;
}
