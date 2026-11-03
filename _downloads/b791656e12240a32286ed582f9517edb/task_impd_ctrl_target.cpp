#include "crobot/control/task_impd_ctrl_target.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
TaskImpdCtrlTargetPtr TaskImpdCtrlTarget::MakeShared(
    ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    std::vector<int> vdir,
    std::vector<int> fdir,
    kdl::CollisionContact::ConstPtr contact) {
  return std::make_shared<TaskImpdCtrlTarget>(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      std::move(vdir),
      std::move(fdir),
      std::move(contact));
}

//==============================================================================
TaskImpdCtrlTarget::TaskImpdCtrlTarget(
    ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    std::vector<int> vdir,
    std::vector<int> fdir,
    kdl::CollisionContact::ConstPtr contact)
  : m_frame_desired(std::move(frame_desired)),
    m_body(std::move(body)),
    m_rotm(rotm),
    m_bpos(bpos),
    m_contact(std::move(contact)) {

  std::sort(vdir.begin(), vdir.end());
  std::sort(fdir.begin(), fdir.end());
  m_nv    = static_cast<int>(vdir.size());
  m_nf    = static_cast<int>(fdir.size());
  m_nctrl = m_nf + m_nv;

  KPv = math::MatrixXd::Identity(m_nv, m_nv) * 2000;
  KDv = math::MatrixXd::Identity(m_nv, m_nv) * 50;
  KPm = math::MatrixXd::Identity(m_nf, m_nf) * 4000;
  KMi = math::MatrixXd::Identity(m_nf, m_nf) * 10;
  KDi = math::MatrixXd::Identity(m_nf, m_nf) * 50;
  KPi = math::MatrixXd::Identity(m_nf, m_nf) * 200;

  m_Sv = math::MatrixX6::Zero(m_nv, 6);
  for (int i = 0; i < m_nv; ++i) {
    m_Sv(i, vdir[i]) = 1;
  }
  m_Sf = math::MatrixX6::Zero(m_nf, 6);
  for (int i = 0; i < m_nf; ++i) {
    m_Sf(i, fdir[i]) = 1;
  }
  m_S                  = math::MatrixX6(m_nctrl, 6);
  m_S.topRows(m_nv)    = m_Sv;
  m_S.bottomRows(m_nf) = m_Sf;
}

//==============================================================================
void TaskImpdCtrlTarget::set_desired_force(const math::VectorXd& sfrc) {
  CROBOT_ASSERT_ERROR(sfrc.rows() == m_nf);
  m_sfrc_desired = sfrc;
}

//==============================================================================
std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd>
TaskImpdCtrlTarget::resolve(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const std::vector<math::Matrix4d>& tforms) const {
  (void)qpos;
  math::Matrix3d xmat            = m_body->get_xmat(tforms, m_rotm);
  math::Matrix3d xmat_desired    = m_frame_desired->get_xmat();
  math::Matrix3d rotm_to_desired = xmat_desired.transpose() * xmat;
  math::Vector3d eulr_to_desired
      = math::Quaternion(rotm_to_desired).as_euler(math::EulerAngleSeq::ZYX);

  math::Matrix6d R    = math::Matrix6d::Zero();
  R.block<3, 3>(0, 0) = xmat_desired;
  R.block<3, 3>(3, 3) = xmat_desired;
  math::Matrix6d T    = math::Matrix6d::Zero();
  T.block<3, 3>(0, 0)
      = eulr_to_desired.omega2eulrdot_mat(math::EulerAngleSeq::ZYX);
  T.block<3, 3>(3, 3) = math::Matrix3d::Identity();

  math::Jacobian jacb     = m_body->get_jacb(tforms);
  math::Jacobian J        = m_body->get_jacb_at_bpos(tforms, jacb, m_bpos);
  math::MatrixXd jacobian = m_S * T * R.transpose() * J;

  math::Vector3d xpos         = m_body->get_xpos(tforms, m_bpos);
  math::Vector3d xpos_desired = m_frame_desired->get_xpos();
  math::Vector3d bpos_to_desired
      = xmat_desired.transpose() * (xpos - xpos_desired);
  math::Vector6d cpos       = math::compose(eulr_to_desired, bpos_to_desired);
  math::VectorXd spos_error = -m_Sv * cpos;

  math::Vector6d xvel = m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);
  math::Vector6d xvel_desired = math::compose(
      m_frame_desired->get_xvelr(), m_frame_desired->get_xvelp());
  math::VectorXd svel_error = m_Sv * T * R.transpose() * (xvel_desired - xvel);

  math::Vector6d xacc_desired = math::compose(
      m_frame_desired->get_xaccr(), m_frame_desired->get_xaccp());
  // Todo: sacc_desired consists of two extra terms
  math::VectorXd sacc_desired = m_Sv * T * R.transpose() * xacc_desired;
  math::VectorXd sacc_resolved_pctrl
      = sacc_desired + KPv * spos_error + KDv * svel_error;
  math::VectorXd sacc_resolved = math::VectorXd::Zero(m_nctrl);
  sacc_resolved += m_S * m_Sv.transpose() * sacc_resolved_pctrl;

  math::VectorXd sacc_velcomps = math::VectorXd::Zero(m_nctrl);
  sacc_resolved += sacc_velcomps;

  // resolve for force related terms
  math::VectorXd qfrc_resolved = m_body->get_rbtree()->make_zero_qvel();

  if (m_contact) {
    math::Vector6d xfrc
        = math::compose(math::Vector3d::Zero(), m_contact->get_xfrc());
    math::Vector6d cfrc          = R.transpose() * xfrc;
    math::Vector6d cfrc_comp     = cfrc;
    math::VectorXd sfrc          = m_Sf * cfrc;
    math::Vector6d cfrc_resolved = cfrc_comp;
    math::VectorXd xfrc_resolved = R * cfrc_resolved;
    qfrc_resolved                = J.transpose() * xfrc_resolved;

    if (m_nf > 0) {
      math::VectorXd spos_error_impd
          = -m_Sf * cpos + KPm.inverse() * m_sfrc_desired;
      math::Vector6d cvel            = T * R.transpose() * xvel;
      math::VectorXd svel_error_impd = -m_Sf * cvel;
      // clang-format off
      math::VectorXd sacc_resolved_ictrl = KMi.inverse() * (
            m_sfrc_desired
          - sfrc
          + KDi * svel_error_impd
          + KPi * spos_error_impd);
      // clang-format on
      sacc_resolved += m_S * m_Sf.transpose() * sacc_resolved_ictrl;
    }
  }

  return std::make_tuple<>(sacc_resolved, qfrc_resolved, jacobian);
}
