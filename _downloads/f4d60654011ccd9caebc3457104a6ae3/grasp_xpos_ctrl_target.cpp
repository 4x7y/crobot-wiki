#include "crobot/control/grasp_xpos_ctrl_target.hpp"

#include <iostream>

#include "crobot/kdl/rigid_body_tree.hpp"

#include "yaml-cpp/yaml.h"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
GraspXposCtrlTargetPtr GraspXposCtrlTarget::MakeShared(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {
  return std::make_shared<GraspXposCtrlTarget>(
      std::move(frame_desired), std::move(body), rotm, bpos, ctrl_param_file);
}

//==============================================================================
GraspXposCtrlTarget::GraspXposCtrlTarget(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file)
  : m_frame_desired(std::move(frame_desired)),
    m_body(std::move(body)),
    m_rotm(rotm),
    m_bpos(bpos) {

  YAML::Node ctrl_params = YAML::LoadFile(ctrl_param_file);

  m_nv = 6;

  KPv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KPv"].as<double>();
  KDv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KDv"].as<double>();
  KIv = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KIv"].as<double>();

  m_spos_err_sum = math::Vector6d::Zero();

  // KPm = math::MatrixXd::Identity(m_nf, m_nf) * 4000;
}

//==============================================================================
// the control is done in the body frame of the screwdriver
std::tuple<math::Vector6d, math::Vector6d> GraspXposCtrlTarget::resolve(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const std::vector<math::Matrix4d>& tforms) {
  (void)qpos;
  math::Matrix3d xmat = m_body->get_xmat(tforms, m_rotm);
  // std::cout << "xmat: " << std::endl
  //           << xmat.rotm2tform().tform2quat().as_euler() << std::endl;
  math::Matrix3d xmat_desired = m_frame_desired->get_xmat();
  // std::cout << "xmat_desired: " << std::endl
  //           << xmat_desired.rotm2tform().tform2quat().as_euler() <<
  //           std::endl;
  math::Matrix3d brotm_desired = xmat.transpose() * xmat_desired;
  // std::cout << "brotm_desired: " << std::endl
  //           << brotm_desired.rotm2tform().tform2quat().as_euler() <<
  //           std::endl;
  math::Vector3d eulr_desired
      = math::Quaternion(brotm_desired).as_euler(math::EulerAngleSeq::ZYX);

  // std::cout << "eulr_desired: " << eulr_desired.transpose() << std::endl;

  math::Matrix6d R    = math::Matrix6d::Zero();
  R.block<3, 3>(0, 0) = xmat;
  R.block<3, 3>(3, 3) = xmat;

  math::Vector3d eulr_T = math::Vector3d::Zero();

  math::Matrix6d T    = math::Matrix6d::Zero();
  T.block<3, 3>(0, 0) = eulr_T.omega2eulrdot_mat(math::EulerAngleSeq::ZYX);
  T.block<3, 3>(3, 3) = math::Matrix3d::Identity();

  // screwdriver's jacobian
  math::Jacobian jacb = m_body->get_jacb(tforms);

  // pos error w.r.t. body frame
  math::Vector3d xpos            = m_body->get_xpos(tforms, m_bpos);
  math::Vector3d xpos_desired    = m_frame_desired->get_xpos();
  math::Vector3d bpos_to_desired = xmat.transpose() * (xpos_desired - xpos);
  math::Vector6d spos_error      = math::compose(eulr_desired, bpos_to_desired);
  // std::cout << "bpos_to_desired: " << bpos_to_desired.transpose() <<
  // std::endl;

  m_spos_err_sum += spos_error;

  // vel error w.r.t. body frame
  math::Vector6d xvel = m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);
  math::Vector6d xvel_desired = math::compose(
      m_frame_desired->get_xvelr(), m_frame_desired->get_xvelp());
  math::VectorXd svel_error = T * R.transpose() * (xvel_desired - xvel);
  // std::cout << "xvel: " << xvel.transpose() << std::endl;
  // std::cout << "xvel_desired: " << xvel_desired.transpose() << std::endl;

  // acc error w.r.t. body frame
  math::Vector6d xacc_desired = math::compose(
      m_frame_desired->get_xaccr(), m_frame_desired->get_xaccp());
  // Todo: sacc_desired consists of two extra terms
  math::Vector6d sacc_desired = T * R.transpose() * xacc_desired;

  // std::cout << "sacc_desired: " << sacc_desired.transpose() << std::endl;
  std::cout << "spos_error: " << spos_error.transpose() << std::endl;
  // std::cout << "svel_error: " << svel_error.transpose() << std::endl;
  // std::cout << "T: \n" << T << std::endl;
  // std::cout << "T.inverse(): \n" << T.inverse() << std::endl;

  // euler angle rates acc -> angular acc
  math::Vector6d sacc_resolved = T.inverse()
                                 * (sacc_desired + KPv * spos_error
                                    + KDv * svel_error + KIv * m_spos_err_sum);
  // std::cout << "sacc_resolved_before_Tinv: "
  //           << (sacc_desired + KPv * spos_error + KDv *
  //           svel_error).transpose()
  //           << std::endl;
  std::cout << "sacc_resolved[rx,ry,rz,x,y,z]: " << sacc_resolved.transpose()
            << std::endl;

  // total wrench <bfrc, btorq> needed for a given acc
  math::Vector6d bwrench = m_body->get_ext_bfrc_btorq_with_bacc(
      tforms, jacb, qvel, sacc_resolved.tail<3>(), sacc_resolved.head<3>());

  // std::cout << "bwrench desired: " << bwrench.transpose() << std::endl;

  // force that should be applied to the body by finger includes the gravity of
  // the screwdriver
  // bwrench.head<3>()
  //     = bwrench.head<3>() - math::Vector3d(0.0, 0.0, -9.8) *
  //     m_body->get_mass();

  // X ddot solved in body frame

  // sacc_resolved = T.inverse() * sacc_desired;
  return std::make_pair(sacc_resolved, bwrench);
}

//==============================================================================
kdl::ConstRigidBodyPtr GraspXposCtrlTarget::get_body() const {
  return m_body;
}
