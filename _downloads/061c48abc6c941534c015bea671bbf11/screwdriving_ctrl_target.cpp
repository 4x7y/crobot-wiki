#include "crobot/control/screwdriving_ctrl_target.hpp"

#include <iostream>

#include "crobot/common/logging.hpp"
#include "crobot/control/admittance_target.hpp"
#include "crobot/control/pid_target.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

#include "yaml-cpp/yaml.h"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
ScrewdrivingCtrlTargetPtr ScrewdrivingCtrlTarget::MakeShared(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {
  return std::make_shared<ScrewdrivingCtrlTarget>(
      std::move(frame_desired), std::move(body), rotm, bpos, ctrl_param_file);
}

//==============================================================================
ScrewdrivingCtrlTarget::ScrewdrivingCtrlTarget(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file)
  : m_frame_desired(std::move(frame_desired)),
    m_body(std::move(body)),
    m_rotm(rotm),
    m_bpos(bpos) {

  m_swap_matrix                   = math::Matrix6d::Zero(6, 6);
  m_swap_matrix.block<3, 3>(0, 3) = math::Matrix3d::Identity();
  m_swap_matrix.block<3, 3>(3, 0) = math::Matrix3d::Identity();

  m_coupling_matrix = math::MatrixXd::Zero(6, 5);

  YAML::Node ctrl_params = YAML::LoadFile(ctrl_param_file);

  m_dt = ctrl_params["ctrl"]["dt"].as<double>();
  m_targets.resize(6);

  YAML::Node targets = ctrl_params["ctrl"]["params"];
  m_force_target_id  = -1;
  for (std::size_t i = 0; i < targets.size(); i++) {
    YAML::Node target        = targets[i];
    YAML::Node active_fields = target["active"];
    std::string type         = target["type"].as<std::string>();

    for (std::size_t j = 0; j < active_fields.size(); j++) {
      int id = active_fields[j].as<int>();

      // check type
      if (type == "ADMITTANCE") {
        m_targets[id] = std::make_shared<AdmittanceTarget>(
            target["m"].as<double>(),
            target["kd"].as<double>(),
            target["ks"].as<double>(),
            target["kp"].as<double>(),
            target["kv"].as<double>(),
            target["ki"].as<double>(),
            m_dt,
            TargetType::ADMITTANCE);
        CROBOT_LOG_INFO("Admittance target is initialized at index {}.", id);
      }
      else if (type == "POSITION") {
        m_targets[id] = std::make_shared<PIDTarget>(
            target["kp"].as<double>(),
            target["kv"].as<double>(),
            target["ki"].as<double>(),
            m_dt,
            TargetType::POSITION);
        CROBOT_LOG_INFO("Position target is initialized at index {}.", id);
      }
      else if (type == "COUPLED") {
        int leader_id = target["leader"].as<int>();
        double coeff  = target["coeff"].as<double>();

        math::MatrixXd tmp_matrix = math::MatrixXd::Identity(6, 6);
        tmp_matrix(id, leader_id) = coeff;
        // delete the column of id and save the rest to m_coupling_matrix
        m_coupling_matrix.block(0, 0, 6, id) = tmp_matrix.block(0, 0, 6, id);
        m_coupling_matrix.block(0, id, 6, 5 - id)
            = tmp_matrix.block(0, id + 1, 6, 5 - id);
        m_coupling_matrix_inv = math::calculate_jacb_inv(m_coupling_matrix);
        CROBOT_LOG_INFO(
            "Coupled target is initialized at index {} with leader {}.",
            id,
            leader_id);
      }
      else if (type == "FORCE") {
        m_force_target_id = id;
        m_targets[id]     = std::make_shared<PIDTarget>(
            target["kp"].as<double>(),
            target["kv"].as<double>(),
            target["ki"].as<double>(),
            m_dt,
            TargetType::FORCE);
        CROBOT_LOG_INFO("Force target is initialized at index %d.", id);
      }
      else {
        // ASSERT ERROR with type
        CROBOT_ASSERT_ERROR(false, "Unknown target type %s.", type.c_str());
      }
    }
  }

  // check if m_targets are all initialized
  for (std::size_t i = 0; i < m_targets.size(); i++) {
    CROBOT_ASSERT_ERROR(
        m_targets[i] != nullptr,
        "Target %d is not initialized.",
        static_cast<int>(i));
  }
}

//==============================================================================
// the control is done in the body frame of the screwdriver
std::tuple<math::Vector6d, math::Vector6d> ScrewdrivingCtrlTarget::resolve(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::Vector6d& fext,
    const math::Vector6d& fext_desired,
    const std::vector<math::Matrix4d>& tforms) {
  (void)qpos;
  math::Matrix3d xmat         = m_body->get_xmat(tforms, m_rotm);
  math::Matrix3d xmat_T       = xmat.transpose();
  math::Matrix3d xmat_desired = m_frame_desired->get_xmat();

  math::Matrix3d brotm_desired = xmat_T * xmat_desired;
  math::Vector3d eulr_desired
      = math::Quaternion(brotm_desired).as_euler(math::EulerAngleSeq::ZYX);

  math::Vector3d eulr_T = math::Vector3d::Zero();
  math::Matrix6d T      = math::Matrix6d::Zero();
  T.block<3, 3>(3, 3)   = eulr_T.omega2eulrdot_mat(math::EulerAngleSeq::ZYX);
  T.block<3, 3>(0, 0)   = math::Matrix3d::Identity();

  math::Matrix6d R    = math::Matrix6d::Zero();
  R.block<3, 3>(0, 0) = xmat;
  R.block<3, 3>(3, 3) = xmat;
  math::Matrix6d R_T  = R.transpose();

  // screwdriver's jacobian
  math::Jacobian jacb_rp = m_body->get_jacb(tforms);
  // swap first 3 rows and last 3 rows through matrix multiplication
  // so the first 3 rows are the linear velocity jacobian
  // and the last 3 rows are the angular velocity jacobian

  math::Jacobian jacb = m_swap_matrix * jacb_rp;

  // translational x & x_d
  math::Vector3d body_xpos = m_body->get_xpos(tforms);
  math::Vector3d bpos = xmat_T * (m_body->get_xpos(tforms, m_bpos) - body_xpos);
  math::Vector3d bpos_desired
      = xmat_T * (m_frame_desired->get_xpos() - body_xpos);

  math::Vector6d spos = math::Vector6d::Zero();
  spos.head<3>()      = bpos;
  // spos.tail<3>() is zero since the controller is in the body frame

  math::Vector6d spos_desired = math::Vector6d::Zero();
  spos_desired.head<3>()      = bpos_desired;
  spos_desired.tail<3>()      = eulr_desired;

  // vel w.r.t. body frame
  math::Vector6d svel
      = T * R_T * m_swap_matrix
        * m_body->get_xvel_at_bpos(tforms, jacb_rp, qvel, m_bpos);
  math::Vector6d svel_desired
      = T * R_T
        * math::compose(
            m_frame_desired->get_xvelp(), m_frame_desired->get_xvelr());

  // acc error w.r.t. body frame
  math::Vector6d xacc_desired = math::compose(
      m_frame_desired->get_xaccp(), m_frame_desired->get_xaccr());
  // Todo: sacc_desired consists of two extra terms
  math::Vector6d sacc_desired = T * R_T * xacc_desired;

  std::cout << "spos: " << spos.transpose() << std::endl;
  std::cout << "spos_desired: " << spos_desired.transpose() << std::endl;

  std::cout << "svel: " << svel.transpose() << std::endl;
  std::cout << "svel_desired: " << svel_desired.transpose() << std::endl;

  std::cout << "sacc_desired: " << sacc_desired.transpose() << std::endl;

  // go through the targets and resolve the acc
  math::VectorXd xacc_resolved(
      m_targets.size() - (m_force_target_id > -1 ? 1 : 0));

  std::cout << "xacc_resolved size: " << xacc_resolved.size() << std::endl;
  double force_resolved = 0.0;
  int xacc_count        = 0;
  for (std::size_t i = 0; i < m_targets.size(); i++) {
    if (m_targets[i]->m_type != TargetType::FORCE) {
      xacc_resolved[xacc_count] = m_targets[i]->generate(
          spos[i],
          svel[i],
          spos_desired[i],
          svel_desired[i],
          sacc_desired[i],
          fext[i]);
      xacc_count++;
    }
    else {
      force_resolved = m_targets[i]->generate(
          fext[i], 0.0, fext_desired[i], 0.0, fext_desired[i], 0.0);
    }
  }

  std::cout << "xacc_resolved: " << xacc_resolved.transpose() << std::endl;
  math::Vector6d sacc_resolved = math::Vector6d::Zero();
  // euler angle rates acc -> angular acc
  // check if m_coupling_matrix is all zero
  if (m_coupling_matrix.isZero()) {
    sacc_resolved = T.inverse() * xacc_resolved;
  }
  else {
    sacc_resolved = T.inverse() * m_coupling_matrix * xacc_resolved;
  }

  // std::cout << "sacc_resolved_before_Tinv: "
  //           << (sacc_desired + KPv * spos_error + KDv *
  //           svel_error).transpose()
  //           << std::endl;
  // std::cout << "sacc_resolved[rx,ry,rz,x,y,z]: " << sacc_resolved.transpose()
  //           << std::endl;

  // total wrench <bfrc, btorq> needed for a given acc
  math::Vector6d bwrench = m_body->get_ext_bfrc_btorq_with_bacc(
      tforms, jacb_rp, qvel, sacc_resolved.head<3>(), sacc_resolved.tail<3>());

  // bwrench += fext;

  // apply the force ctrl result
  if (m_force_target_id != -1) {
    bwrench[m_force_target_id] = force_resolved;
  }

  // this is the total force

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
kdl::ConstRigidBodyPtr ScrewdrivingCtrlTarget::get_body() const {
  return m_body;
}
