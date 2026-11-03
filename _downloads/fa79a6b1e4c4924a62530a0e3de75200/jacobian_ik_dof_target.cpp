#include "crobot/ik/jacobian_ik_dof_target.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
JacobianIKDofTarget::JacobianIKDofTarget(
    kdl::ConstFramePtr desired_frame,
    kdl::ConstRigidBodyPtr body,
    const std::vector<int>& dofs,
    kdl::BodyContact::ConstPtr contact,
    math::EulerAngleSeq euler_seq)
  : JacobianIKTarget(),
    m_desired_frame(desired_frame),
    m_body(body),
    m_dofs(dofs),
    m_contact(contact),
    m_euler_seq(euler_seq) {
  m_qvel_addrs = body->get_support_qvel_addr();
}

//==============================================================================
JacobianIKDofTarget::JacobianIKDofTarget(
    kdl::ConstFramePtr desired_frame,
    kdl::ConstRigidBodyPtr body,
    const std::vector<int>& dofs,
    math::EulerAngleSeq euler_seq,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm)
  : JacobianIKTarget(),
    m_desired_frame(desired_frame),
    m_body(body),
    m_dofs(dofs),
    m_euler_seq(euler_seq),
    m_bpos(bpos),
    m_rotm(rotm) {
  m_qvel_addrs = body->get_support_qvel_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::MatrixXd>
JacobianIKDofTarget::compute_value_and_jacobian(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix3d xmat_desired = m_desired_frame->get_xmat();
  math::Vector3d xpos_desired = m_desired_frame->get_xpos();

  math::Vector3d bpos_target;
  math::Matrix3d rotm_target;
  if (m_contact != nullptr) {
    kdl::BodyFrame::Ptr frame_target = m_contact->get_frame2();
    bpos_target                      = frame_target->get_bpos();
    rotm_target                      = frame_target->get_rotm();
  }
  else {
    bpos_target = m_bpos;
    rotm_target = m_rotm;
  }

  math::Matrix3d xmat            = m_body->get_xmat(tforms, rotm_target);
  math::Vector3d xpos            = m_body->get_xpos(tforms, bpos_target);
  math::Matrix3d rotm_to_desired = xmat_desired.transpose() * xmat;
  math::Vector3d eulrdiff
      = math::Quaternion(rotm_to_desired).as_euler(m_euler_seq);
  math::Vector3d posdiff = xmat_desired.transpose() * (xpos - xpos_desired);
  math::Vector6d dx      = -math::compose(eulrdiff, posdiff);
  math::VectorXd dx_scon = dx(m_dofs);

  math::Matrix6d R    = math::Matrix6d::Zero();
  R.block<3, 3>(0, 0) = xmat_desired;
  R.block<3, 3>(3, 3) = xmat_desired;
  math::Matrix6d T    = math::Matrix6d::Zero();
  T.block<3, 3>(0, 0) = eulrdiff.omega2eulrdot_mat(m_euler_seq);
  T.block<3, 3>(3, 3) = math::Matrix3d::Identity();

  math::Jacobian jacb_body = m_body->get_jacb(tforms);
  math::Jacobian jacb
      = m_body->get_jacb_at_bpos(tforms, jacb_body, bpos_target);
  math::Jacobian jacb_eulr = T * R.transpose() * jacb;
  math::MatrixXd jacb_scon = jacb_eulr(m_dofs, Eigen::all);

  return std::make_pair(dx_scon, jacb_scon);
}

//==============================================================================
int JacobianIKDofTarget::get_ndofs() const {
  return m_dofs.size();
}

//==============================================================================
std::vector<int> JacobianIKDofTarget::get_qvel_addrs() const {
  return m_qvel_addrs;
}
