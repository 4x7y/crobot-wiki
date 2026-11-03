#include "crobot/ik/jacobian_ik_pose_target.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
JacobianIKPoseTarget::JacobianIKPoseTarget(
    kdl::ConstFramePtr desired_frame,
    kdl::ConstRigidBodyPtr body,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm)
  : JacobianIKTarget(),
    m_desired_frame(desired_frame),
    m_body(body),
    m_bpos(bpos),
    m_rotm(rotm) {
  m_qvel_addrs = body->get_support_qvel_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::MatrixXd>
JacobianIKPoseTarget::compute_value_and_jacobian(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix3d xmat_desired = m_desired_frame->get_xmat();
  // math::Matrix6d xmat_desired_diag          = math::Matrix6d::Zero();
  // xmat_desired_diag.topLeftCorner(3, 3)     = xmat_desired;
  // xmat_desired_diag.bottomRightCorner(3, 3) = xmat_desired;
  math::Vector3d xpos_desired = m_desired_frame->get_xpos();
  math::Quaternion xquat_desired(xmat_desired);

  math::Quaternion xquat   = m_body->get_xquat(tforms);
  math::Vector3d xpos      = m_body->get_xpos(tforms, m_bpos);
  math::Jacobian jacb_body = m_body->get_jacb(tforms);
  math::Jacobian jacb = m_body->get_jacb_at_bpos(tforms, jacb_body, m_bpos);

  math::Quaternion quatdiff = xquat_desired * xquat.conjugate();
  math::AngleAxisd axang    = quatdiff.as_axang();
  math::Vector3d angdiff    = axang.axis() * axang.angle();
  math::Vector3d posdiff    = xpos_desired - xpos;
  math::Vector6d dx         = math::compose(angdiff, posdiff);

  return std::make_pair(dx, jacb);
}

//==============================================================================
int JacobianIKPoseTarget::get_ndofs() const {
  return 6;
}

//==============================================================================
std::vector<int> JacobianIKPoseTarget::get_qvel_addrs() const {
  return m_qvel_addrs;
}
