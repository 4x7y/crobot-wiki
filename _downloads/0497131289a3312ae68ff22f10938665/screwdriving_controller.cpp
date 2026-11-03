#include "crobot/control/screwdriving_controller.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;
using namespace math;

//==============================================================================
ScrewdrivingControllerPtr ScrewdrivingController::MakeShared(
    kdl::ConstRigidBodyTreePtr rbtree, RollingGrasp::Ptr grasp) {
  return std::make_shared<ScrewdrivingController>(std::move(rbtree), grasp);
}

//==============================================================================
ScrewdrivingController::ScrewdrivingController(
    ConstRigidBodyTreePtr rbtree, RollingGrasp::Ptr grasp)
  : m_rbtree(std::move(rbtree)), m_grasp(grasp) {
  CROBOT_ASSERT_ERROR(m_rbtree);
  auto qpos_limits             = m_rbtree->get_qpos_limits();
  m_null_space_obj.qpos_lb     = qpos_limits.first;
  m_null_space_obj.qpos_ub     = qpos_limits.second;
  m_null_space_obj.qpos_center = (qpos_limits.first + qpos_limits.second) / 2.0;
}

//==============================================================================
math::VectorXd ScrewdrivingController::compute(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::Vector6d& fext_desired,
    const std::vector<math::Vector3d>& frc_internal) const {
  int nqvel = m_rbtree->get_qvel_dim();
  int nctc  = m_grasp->get_contacts().size();

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  std::vector<RollingContact::Ptr> contacts = m_grasp->get_contacts();

  math::MatrixXd J = math::MatrixXd::Zero(3 * nctc, nqvel);
  math::MatrixXd R = math::MatrixXd::Zero(3 * nctc, 3 * nctc);

  // get the grasp matrix 6 x 3*nctc, transform force exerted at each contact
  // point (in contact frame1) to the wrench exerted on the object (in object
  // frame)
  math::MatrixXd G = m_grasp->get_grasp_matrix();

  math::VectorXd cfrc_internal = math::VectorXd::Zero(3 * nctc);

  // sacc is resolved in the target's body frame
  math::Vector6d sacc_resolved;
  math::Vector6d bwrench_resolved;

  math::VectorXd ctc_cfrc = m_grasp->get_ctc_cfrc_stack(tforms);
  std::cout << "ctc_cfrc = " << ctc_cfrc.transpose() << std::endl;

  math::Vector6d fext = G * ctc_cfrc; // manipulator to the object

  std::cout << "fext in the body frame = " << fext.transpose() << std::endl;
  std::cout << "fext_desired = " << fext_desired.transpose() << std::endl;

  std::tie(sacc_resolved, bwrench_resolved)
      = m_target->resolve(qpos, qvel, fext, fext_desired, tforms);
  math::VectorXd qfrc_resolved = math::VectorXd::Zero(nqvel);

  std::cout << "\nsacc_resolved = " << sacc_resolved.transpose() << std::endl;
  std::cout << "\nbwrench_desired = " << bwrench_resolved.transpose()
            << std::endl;

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

    cfrc_internal.segment<3>(3 * ctc_id) = frc_internal[ctc_id];
    ctc_id++;
  }

  // solve for the pesudo inverse of J
  math::MatrixXd J_pinv = math::calculate_jacb_inv(J);

  // due to drivative of X_dot
  math::VectorXd resolved_xacc = R * G.transpose() * sacc_resolved;

  // (4.10) in Cole's rolling paper
  math::VectorXd qacc_resolved = J_pinv * (resolved_xacc);

  // joint torque due to qacc
  qfrc_resolved += m_rbtree->inverse_dynamics(qpos, qvel, qacc_resolved);

  // std::cout << "\nqfrc due to qacc = " << qfrc_resolved.transpose()
  //           << std::endl;

  // return qfrc_resolved;

  //============================================================================
  // joint torque due to external force bwrench_resolved
  int num_wrench_biasis    = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp = m_grasp->get_grasp_matrix();
  // math::MatrixX6 mat_grasp_pinv
  //     = mat_grasp.completeOrthogonalDecomposition().pseudoInverse();
  math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);

  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;

  math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;
  // std::cout << "\ncwrench_resolved = " << cwrench_resolved.transpose()
  //           << std::endl;
  math::VectorXd cfrc_internal_null = mat_null_space_mapping * cfrc_internal;
  // std::cout << "\ncfrc_internal_null = " << cfrc_internal_null.transpose()
  //           << std::endl;
  cwrench_resolved += cfrc_internal_null;

  // math::VectorXd qfrc_resolved_null = J.transpose() * R * cfrc_internal_null;

  qfrc_resolved += J.transpose() * R * cwrench_resolved;

  // return qfrc_resolved;

  // // null space control, to minimize the movement of the WAM
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
ScrewdrivingCtrlTargetPtr ScrewdrivingController::add_screwdriving_ctrl_target(
    kdl::ConstFramePtr desired_frame,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {

  m_target = ScrewdrivingCtrlTarget::MakeShared(
      std::move(desired_frame), std::move(body), rotm, bpos, ctrl_param_file);

  return m_target;
}
