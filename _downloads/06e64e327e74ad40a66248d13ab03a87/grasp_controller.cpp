#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/control/grasp_controller.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;
using namespace math;

//==============================================================================
GraspControllerPtr GraspController::MakeShared(
    kdl::ConstRigidBodyTreePtr rbtree,
    RollingGrasp::Ptr grasp,
    math::Vector3d& frc_norm) {
  return std::make_shared<GraspController>(std::move(rbtree), grasp, frc_norm);
}

//==============================================================================
GraspController::GraspController(
    ConstRigidBodyTreePtr rbtree,
    RollingGrasp::Ptr grasp,
    math::Vector3d& frc_norm)
  : m_rbtree(std::move(rbtree)), m_grasp(grasp), m_frc_norm(frc_norm) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  auto qpos_limits             = m_rbtree->get_qpos_limits();
  m_null_space_obj.qpos_lb     = qpos_limits.first;
  m_null_space_obj.qpos_ub     = qpos_limits.second;
  m_null_space_obj.qpos_center = (qpos_limits.first + qpos_limits.second) / 2.0;
}

//==============================================================================
math::VectorXd GraspController::compute(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const std::vector<math::Vector3d>& frc_norm) const {
  int nqvel = m_rbtree->get_qvel_dim();
  int nctc  = m_grasp->get_contacts().size();

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  std::vector<RollingContact::Ptr> contacts = m_grasp->get_contacts();

  math::Jacobian jacb_target = m_target->get_body()->get_jacb(tforms);
  math::Matrix3d xmat_target = m_target->get_body()->get_xmat(tforms);
  math::Vector6d xvel_target
      = m_target->get_body()->get_xvel(jacb_target, qvel);
  math::Vector6d bvel_target;
  bvel_target << xmat_target.transpose() * xvel_target.head(3),
      xmat_target.transpose() * xvel_target.tail(3);

  math::MatrixXd J     = math::MatrixXd::Zero(3 * nctc, nqvel);
  math::MatrixXd R     = math::MatrixXd::Zero(3 * nctc, 3 * nctc);
  math::MatrixXd R_dot = math::MatrixXd::Zero(3 * nctc, 3 * nctc);

  math::VectorXd cfrc_internal = math::VectorXd::Zero(3 * nctc);

  // sacc is resolved in the target's body frame
  math::Vector6d sacc_resolved;
  math::Vector6d bwrench_resolved;
  std::tie(sacc_resolved, bwrench_resolved)
      = m_target->resolve(qpos, qvel, tforms);
  math::VectorXd qfrc_resolved = math::VectorXd::Zero(nqvel);

  // std::cout << "\nsacc_resolved = " << sacc_resolved.transpose() <<
  // std::endl;
  // std::cout << "\nbwrench_desired = " << bwrench_resolved.transpose()
  //           << std::endl;

  // Construct J, jocabian that transforms qvel to [v_1, v_2, ..., v_nctc]T
  // R, rotation matrix that transforms [v_1, v_2, ..., v_nctc]T from the
  // concact frame to the world frame
  int row_J  = 0;
  int ctc_id = 0;

  for (RollingContact::Ptr contact : contacts) {
    ConstRigidBodyPtr body_finger = contact->get_body2();
    math::Jacobian jacb_body      = body_finger->get_jacb(tforms);
    math::LinearJacobian jacp_ctc = body_finger->get_jacp_at_bpos(
        tforms, jacb_body, contact->get_frame2()->get_bpos());
    J.middleRows(row_J, jacp_ctc.rows()) = jacp_ctc;
    row_J += jacp_ctc.rows();

    math::Matrix3d xmat_ctc               = contact->get_xmat1(tforms);
    R.block<3, 3>(3 * ctc_id, 3 * ctc_id) = xmat_ctc;

    // you wen ti !!!!!
    // R_dot.block<3, 3>(3 * ctc_id, 3 * ctc_id)
    //     = contact->get_xvelr_ctc1(tforms).skew() * xmat_ctc;

    cfrc_internal.segment<3>(3 * ctc_id) = frc_norm[ctc_id];
    ctc_id++;
  }

  // get the grasp matrix 6 x 3*nctc, transform force exerted at each contact
  // point (in contact frame1) to the wrench exerted on the object (in object
  // frame)
  math::MatrixXd G     = m_grasp->get_grasp_matrix();
  math::MatrixXd G_dot = m_grasp->get_grasp_matrix_deriv();

  // solve for the pesudo inverse of J
  math::MatrixXd J_pinv = math::calculate_jacb_inv(J);

  // exchange the first 3 rows of the last 3 rows, because for G the first 3
  // rows correspond to the force/velp
  sacc_resolved = math::Vector6d(
      sacc_resolved[3],
      sacc_resolved[4],
      sacc_resolved[5],
      sacc_resolved[0],
      sacc_resolved[1],
      sacc_resolved[2]);
  bvel_target = math::Vector6d(
      bvel_target[3],
      bvel_target[4],
      bvel_target[5],
      bvel_target[0],
      bvel_target[1],
      bvel_target[2]);

  // due to drivative of X_dot
  math::VectorXd resolved_xacc = R * G.transpose() * sacc_resolved;
  // std::cout << "\nx_acc resolved = " << resolved_xacc.transpose() <<
  // std::endl; std::cout << "\nqacc xdot = " << (J_pinv *
  // resolved_xacc).transpose()
  //           << std::endl;

  // due to drivative of G
  // math::VectorXd resolved_gdot = R * G_dot.transpose() * bvel_target;
  // std::cout << "\ndue to drivative of G = " << resolved_gdot.transpose()
  //           << std::endl;
  // std::cout << "\nqacc gdot = " << (J_pinv * resolved_gdot).transpose()
  //           << std::endl;

  // due to derivative of R
  // math::VectorXd resolved_rdot = R_dot * G.transpose() * bvel_target;
  // std::cout << "\ndue to derivative of R = " << resolved_rdot.transpose()
  //           << std::endl;
  // std::cout << "\nqacc rdot = " << (J_pinv * resolved_rdot).transpose()
  //           << std::endl;

  // (4.10) in Cole's rolling paper
  // math::VectorXd qacc_resolved
  //     = J_pinv * (resolved_xacc + resolved_gdot + resolved_rdot);
  math::VectorXd qacc_resolved = J_pinv * (resolved_xacc);

  // joint torque due to qacc
  qfrc_resolved += m_rbtree->inverse_dynamics(qpos, qvel, qacc_resolved);

  // std::cout << "\nqfrc due to qacc = " << qfrc_resolved.transpose()
  //           << std::endl;

  // return qfrc_resolved;

  //============================================================================
  // joint torque due to external force bwrench_resolved
  int num_wrench_biasis         = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp      = m_grasp->get_grasp_matrix();
  math::Matrix6X mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;

  math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;
  // std::cout << "\ncwrench_resolved = " << cwrench_resolved.transpose()
  //           << std::endl;
  math::VectorXd cfrc_internal_null = mat_null_space_mapping * cfrc_internal;
  // std::cout << "\ncfrc_internal = " << cfrc_internal.transpose() <<
  // std::endl; std::cout << "\ncfrc_internal_null = " <<
  // cfrc_internal_null.transpose()
  //           << std::endl;
  cwrench_resolved += cfrc_internal_null;

  math::VectorXd qfrc_resolved_null = J.transpose() * R * cfrc_internal_null;
  // std::cout << "\nqfrc_resolved_null = " << qfrc_resolved_null.transpose()
  //           << std::endl;

  qfrc_resolved += J.transpose() * R * cwrench_resolved;

  // null space control, to minimize the movement of the WAM
  math::MatrixXd eye            = math::MatrixXd::Identity(nqvel, nqvel);
  math::MatrixXd J_null         = eye - J_pinv * J;
  math::VectorXd qvel_potential = math::VectorXd::Zero(nqvel);

  std::vector<std::string> joint_names_sup;
  joint_names_sup.push_back("bt_WAMJ1");
  joint_names_sup.push_back("bt_WAMJ2");
  joint_names_sup.push_back("bt_WAMJ3");
  joint_names_sup.push_back("bt_WAMJ4");
  // joint_names_sup.push_back("rh_WRJ2");
  // joint_names_sup.push_back("rh_WRJ1");

  std::vector<int> joint_sup_addr = m_rbtree->get_qvel_addrs(joint_names_sup);
  for (std::size_t i = 0; i < joint_sup_addr.size(); i++) {
    qvel_potential(joint_sup_addr[i]) = -qvel(joint_sup_addr[i]) * 0.001;
  }

  math::VectorXd ctrl = qfrc_resolved + J_null * qvel_potential;

  // std::cout << "\nctrl = " << ctrl.transpose() << std::endl;
  return ctrl;
}

//==============================================================================
GraspXposCtrlTargetPtr GraspController::add_grasp_xpos_target(
    kdl::ConstFramePtr desired_frame,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {
  m_target = GraspXposCtrlTarget::MakeShared(
      std::move(desired_frame), std::move(body), rotm, bpos, ctrl_param_file);

  return m_target;
}
