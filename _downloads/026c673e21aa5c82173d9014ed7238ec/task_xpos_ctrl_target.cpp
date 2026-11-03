#include <iostream>

#include "crobot/control/task_xpos_ctrl_target.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
TaskXposCtrlTargetPtr TaskXposCtrlTarget::MakeShared(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    kdl::CollisionContact::ConstPtr contact) {
  return std::make_shared<TaskXposCtrlTarget>(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      std::move(contact));
}

//==============================================================================
TaskXposCtrlTarget::TaskXposCtrlTarget(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    kdl::CollisionContact::ConstPtr contact)
  : m_frame_desired(std::move(frame_desired)),
    m_body(std::move(body)),
    m_rotm(rotm),
    m_bpos(bpos),
    m_contact(std::move(contact)) {

  KDo = 20.0 * math::Matrix3d::Identity();
  KPo = 100.0 * math::Matrix3d::Identity();
  KDp = 20.0 * math::Matrix3d::Identity();
  KPp = 100.0 * math::Matrix3d::Identity();
}

//==============================================================================
std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd>
TaskXposCtrlTarget::resolve(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const std::vector<math::Matrix4d>& tforms) const {
  (void)qpos;
  math::Matrix6X jacb  = m_body->get_jacb(tforms);
  math::Vector3d xpos  = m_body->get_xpos(tforms, m_bpos);
  math::Matrix3d xmat  = m_body->get_xmat(tforms, m_rotm);
  math::Vector6d xvel  = m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);
  math::Vector3d omega = xvel.segment(0, 3);
  math::Vector3d xvelp = xvel.segment(3, 3);

  math::Matrix3d xmat_d     = m_frame_desired->get_xmat();
  math::Matrix3d xmat_ed    = xmat.transpose() * xmat_d;
  math::AngleAxisd aaxis_de = math::AngleAxisd(xmat_ed);
  math::Vector3d o_de = xmat * std::sin(aaxis_de.angle()) * aaxis_de.axis();

  math::Vector3d ne = xmat.col(0);
  math::Vector3d se = xmat.col(1);
  math::Vector3d ae = xmat.col(2);
  math::Vector3d nd = xmat_d.col(0);
  math::Vector3d sd = xmat_d.col(1);
  math::Vector3d ad = xmat_d.col(2);

  math::Matrix3d L = -0.5
                     * (nd.skew() * ne.skew() + sd.skew() * se.skew()
                        + ad.skew() * ae.skew());
  math::Vector3d omega_d = m_frame_desired->get_xvelr();
  math::Vector3d odot_de = L.transpose() * omega_d - L * omega;
  math::Vector3d alpha_d = m_frame_desired->get_xaccr();
  math::Vector3d alpha_resolved
      = L.inverse()
        * (L.transpose() * alpha_d
           // + Ldot.transpose() * omega_d - Ldot *  omega
           + KDo * odot_de + KPo * o_de);

  math::Vector3d xpos_d  = m_frame_desired->get_xpos();
  math::Vector3d xvelp_d = m_frame_desired->get_xvelp();
  math::Vector3d xaccp_d = m_frame_desired->get_xaccp();

  math::Vector3d xaccp_resolved
      = xaccp_d + KDp * (xvelp_d - xvelp) + KPp * (xpos_d - xpos);
  math::Vector6d xacc_resolved = math::compose(alpha_resolved, xaccp_resolved);

  math::VectorXd qfrc_resolved;
  if (m_contact && m_contact->is_active()) {
    math::Vector3d xfrc = m_contact->get_xfrc();
    math::LinearJacobian jacp_ctc
        = m_body->get_jacp_at_xpos(tforms, jacb, m_contact->get_xpos());
    qfrc_resolved = jacp_ctc.transpose() * xfrc;
  }
  else {
    qfrc_resolved = m_body->get_rbtree()->make_zero_qvel();
  }

  return std::make_tuple<>(xacc_resolved, qfrc_resolved, jacb);
}
