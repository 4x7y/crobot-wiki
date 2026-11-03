#include "crobot/control/obj_xpos_ctrl_target.hpp"

#include <iostream>

#include "crobot/common/logging.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

#include "yaml-cpp/yaml.h"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
ObjXposCtrlTargetPtr ObjXposCtrlTarget::MakeShared(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {
  return std::make_shared<ObjXposCtrlTarget>(
      std::move(frame_desired), std::move(body), rotm, bpos, ctrl_param_file);
}

//==============================================================================
ObjXposCtrlTarget::ObjXposCtrlTarget(
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

  KPv  = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KPv"].as<double>();
  KDv  = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KDv"].as<double>();
  KIv  = math::MatrixXd::Identity(m_nv, m_nv) * ctrl_params["KIv"].as<double>();
  m_dt = ctrl_params["dt"].as<double>();

  // KPm = math::MatrixXd::Identity(m_nf, m_nf) * 4000;

  std::string ctrl_site_name = ctrl_params["ctrl_target"].as<std::string>();
  m_tform_ctrl               = m_body->get_site(ctrl_site_name)->get_tform();

  m_decay         = ctrl_params["decay"].as<double>();
  m_xform_last_fb = math::Matrix4d::Zero();
  m_decay_factor  = 1.0;
}

//==============================================================================
// the control is done in the body frame of the screwdriver
math::Matrix4d ObjXposCtrlTarget::resolve(
    const math::Matrix4d& xform_actual, const math::Matrix4d& xform_cmd) {

  math::Matrix4d xform_actual_ctrl = xform_actual * m_tform_ctrl;
  math::Matrix4d xform_cmd_ctrl    = xform_cmd * m_tform_ctrl;

  math::Matrix3d xmat         = xform_actual_ctrl.block<3, 3>(0, 0);
  math::Matrix3d xmat_desired = m_frame_desired->get_xmat();

  math::Matrix3d brotm_desired = xmat.transpose() * xmat_desired;

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

  // pos error w.r.t. body frame
  math::Vector3d xpos            = xform_actual_ctrl.block<3, 1>(0, 3);
  math::Vector3d xpos_desired    = m_frame_desired->get_xpos();
  math::Vector3d bpos_to_desired = xmat.transpose() * (xpos_desired - xpos);
  math::Vector6d spos_error      = math::compose(eulr_desired, bpos_to_desired);

  math::Vector6d xvel_desired = math::compose(
      m_frame_desired->get_xvelr(), m_frame_desired->get_xvelp());

  // euler angle rates acc -> angular acc
  math::Vector6d sacc_resolved
      = T.inverse() * (m_decay_factor * KPv * spos_error);

  // math::Vector6d xvel_current
  //     = m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);

  math::Vector6d xvel_next = xvel_desired + R * (sacc_resolved * m_dt);

  // calculate xpos and xmat in the next time step from xvel_next
  math::Vector3d xpos_next
      = xform_cmd_ctrl.block<3, 1>(0, 3) + xvel_next.tail<3>() * m_dt;

  math::Quaternion xquat_cmd
      = math::Quaternion(xform_cmd_ctrl.block<3, 3>(0, 0));
  math::Quaternion quat_dot
      = math::Quaternion(
            0, 0.5 * xvel_next[0], 0.5 * xvel_next[1], 0.5 * xvel_next[2])
        * xquat_cmd;

  math::Matrix3d xmat_next
      = (xquat_cmd + quat_dot * m_dt).normalized().toRotationMatrix();

  // math::Matrix3d xmat_next = math::Quaternion(xform_cmd_ctrl.block<3,
  // 3>(0, 0))
  //                                .integrated_worldvel(xvel_next.head<3>(),
  //                                m_dt) .toRotationMatrix();

  math::Matrix4d xform_next    = math::Matrix4d::Identity();
  xform_next.block<3, 3>(0, 0) = xmat_next;
  xform_next.block<3, 1>(0, 3) = xpos_next;

  xform_next = xform_next * m_tform_ctrl.tform_inv();

  // update decay factor
  if (m_xform_last_fb.isZero()) {
    m_xform_last_fb = xform_actual;
  }

  if (m_xform_last_fb.isApprox(xform_actual)) {
    m_decay_factor *= m_decay;
    // if (m_decay_factor < 0.5) {
    //   CROBOT_LOG_INFO("Decay factor: {}", m_decay_factor);
    // }
  }
  else {
    m_decay_factor = 1.0;
  }
  m_xform_last_fb = xform_actual;

  return xform_next;
}

//==============================================================================
kdl::ConstRigidBodyPtr ObjXposCtrlTarget::get_body() const {
  return m_body;
}
