#include "crobot/control/task_xfrc_ctrl_target.hpp"

#include <yaml-cpp/yaml.h>

#include "crobot/common/assert.hpp"
#include "crobot/config.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
TaskXfrcCtrlTargetPtr TaskXfrcCtrlTarget::MakeShared(
    ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    std::vector<int> vdir,
    std::vector<int> fdir,
    kdl::CollisionContact::ConstPtr contact) {
  return std::make_shared<TaskXfrcCtrlTarget>(
      std::move(frame_desired),
      std::move(body),
      rotm,
      bpos,
      std::move(vdir),
      std::move(fdir),
      std::move(contact));
}

//==============================================================================
TaskXfrcCtrlTarget::TaskXfrcCtrlTarget(
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

  std::string ctrl_params_file_name
      = CROBOT_RESOURCE_LOCAL_PATH "control_params/task_xfrc_ctrl_params.yaml";

  YAML::Node ctrl_params = YAML::LoadFile(ctrl_params_file_name);

  std::sort(vdir.begin(), vdir.end());
  std::sort(fdir.begin(), fdir.end());
  m_nv    = static_cast<int>(vdir.size());
  m_nf    = static_cast<int>(fdir.size());
  m_nctrl = m_nf + m_nv;

  KPv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KPv"].as<double>();
  KDv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KDv"].as<double>();
  KIv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KIv"].as<double>();

  m_spos_error_int = math::VectorXd::Zero(m_nv);

  m_S = math::MatrixX6(m_nctrl, 6);
  if (m_nv > 0) {
    m_Sv = math::MatrixX6::Zero(m_nv, 6);
    for (int i = 0; i < m_nv; ++i) {
      m_Sv(i, vdir[i]) = 1;
    }
    m_S.topRows(m_nv) = m_Sv;
  }
  if (m_nf > 0) {
    m_Sf  = math::MatrixX6::Zero(m_nf, 6);
    m_SfC = math::MatrixX6::Zero(6 - m_nf, 6);
    for (int i = 0, j = 0; i < 6; ++i) {
      if (j < m_nf && fdir[j] == i) {
        m_Sf(j, i) = 1;
        j++;
      }
      else {
        m_SfC(i - j, i) = 1;
      }
    }
    m_S.bottomRows(m_nf) = m_Sf;
    // initialize the integration of force error for force PI control
    m_sfrc_error_int = math::VectorXd::Zero(m_nf);
  }

  m_dt = 0.002;
  KPf  = 0.8;
  KIf  = 50;
}

//==============================================================================
void TaskXfrcCtrlTarget::set_desired_force(const math::VectorXd& sfrc) {
  CROBOT_ASSERT_ERROR(sfrc.rows() == m_nf);
  m_sfrc_desired = sfrc;
}

//==============================================================================
std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd>
TaskXfrcCtrlTarget::resolve(
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

  m_spos_error_int += m_dt * spos_error;

  math::Vector6d xvel = m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);
  math::Vector6d xvel_desired = m_frame_desired->get_xvel();
  math::VectorXd svel_error = m_Sv * T * R.transpose() * (xvel_desired - xvel);

  math::Vector6d xacc_desired = m_frame_desired->get_xacc();
  // Todo: sacc_desired consists of two extra terms
  math::VectorXd sacc_desired        = m_Sv * T * R.transpose() * xacc_desired;
  math::VectorXd sacc_resolved_pctrl = sacc_desired + KPv * spos_error
                                       + KDv * svel_error
                                       + KIv * m_spos_error_int;
  math::VectorXd sacc_resolved = m_S * m_Sv.transpose() * sacc_resolved_pctrl;

  // resolve for force related terms
  math::VectorXd qfrc_resolved;
  if (m_contact && m_nf > 0) {
    math::Vector6d xfrc_ee
        = math::compose(math::Vector3d::Zero(), m_contact->get_xfrc());

    math::Vector6d cfrc_ee       = R.transpose() * xfrc_ee;
    math::VectorXd cfrc_ee_error = m_sfrc_desired - m_Sf * cfrc_ee;
    m_sfrc_error_int += m_dt * cfrc_ee_error;
    math::VectorXd cfrc_ee_regulated
        = m_sfrc_desired + KPf * cfrc_ee_error + KIf * m_sfrc_error_int;
    math::Vector6d cfrc_resolved = m_Sf.transpose() * cfrc_ee_regulated
                                   + m_SfC.transpose() * m_SfC * cfrc_ee;
    math::Vector6d xfrc_resolved = R * cfrc_resolved;
    qfrc_resolved                = J.transpose() * xfrc_resolved;
  }
  else {
    qfrc_resolved = m_body->get_rbtree()->make_zero_qvel();
  }

  return std::make_tuple<>(sacc_resolved, qfrc_resolved, jacobian);
}
