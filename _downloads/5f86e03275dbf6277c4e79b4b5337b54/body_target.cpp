#include "crobot/control/body_target.hpp"

#include <iostream>

#include "crobot/common/logging.hpp"
#include "crobot/control/admittance_target.hpp"
#include "crobot/control/couple_target.hpp"
#include "crobot/control/pid_target.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

#include "yaml-cpp/yaml.h"

using namespace crobot;
using namespace control;
using namespace kdl;

//==============================================================================
BodyTargetPtr BodyTarget::MakeShared(
    kdl::ConstFramePtr frame_desired,
    kdl::ConstRigidBodyPtr body,
    const math::Matrix3d& rotm,
    const math::Vector3d& bpos,
    const std::string& ctrl_param_file) {
  return std::make_shared<BodyTarget>(
      std::move(frame_desired), std::move(body), rotm, bpos, ctrl_param_file);
}

//==============================================================================
BodyTarget::BodyTarget(
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

  m_dt = ctrl_params["ctrl"]["dt"].as<double>();
  m_targets.resize(6);

  YAML::Node targets = ctrl_params["ctrl"]["params"];

  for (std::size_t i = 0; i < targets.size(); i++) {
    YAML::Node target        = targets[i];
    YAML::Node active_fields = target["active"];
    std::string type         = target["type"].as<std::string>();

    for (std::size_t j = 0; j < active_fields.size(); j++) {
      int id = active_fields[j].as<int>();
      CROBOT_ASSERT_ERROR(id >= 0 && id < 6, "Invalid target index.");
      CROBOT_ASSERT_ERROR(
          m_targets[id] == nullptr, "Target index is already used.");

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

        CROBOT_ASSERT_ERROR(
            leader_id >= 0 && leader_id < 3 && id >= 3 && id < 6,
            "Leader id %d is out of range. Only the translation follows the "
            "rotation.",
            leader_id);

        m_targets[id] = std::make_shared<CoupleTarget>(
            id, leader_id, coeff, TargetType::COUPLED);

        CROBOT_LOG_INFO(
            "Coupled target is initialized at index {} with leader {}.",
            id,
            leader_id);
      }
      else if (type == "FORCE") {
        m_targets[id] = std::make_shared<PIDTarget>(
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
std::tuple<math::Vector6d, math::Vector6d> BodyTarget::resolve(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::Vector6d& xwrench,
    const std::vector<math::Matrix4d>& tforms) const {
  (void)qpos;
  math::Matrix3d xmat         = m_body->get_xmat(tforms, m_rotm);
  math::Matrix3d xmat_T       = xmat.transpose();
  math::Matrix3d xmat_desired = m_frame_desired->get_xmat();

  math::Matrix3d brotm_desired = xmat_T * xmat_desired;
  math::Vector3d eulr_desired
      = math::Quaternion(brotm_desired).as_euler(math::EulerAngleSeq::XYZ);

  math::Matrix6d T = math::Matrix6d::Zero();
  // math::Vector3d eulr_T = math::Vector3d::Zero();
  T.block<3, 3>(0, 0)
      = eulr_desired.omega2eulrdot_mat(math::EulerAngleSeq::XYZ);
  T.block<3, 3>(3, 3) = math::Matrix3d::Identity();

  math::Matrix6d R    = math::Matrix6d::Zero();
  R.block<3, 3>(0, 0) = xmat;
  R.block<3, 3>(3, 3) = xmat;
  math::Matrix6d R_T  = R.transpose();

  math::Vector6d bwrench_sensed  = R_T * xwrench;
  math::Vector6d bwrench_desired = R_T * m_frame_desired->get_xwrench_ext();

  // screwdriver's jacobian
  math::Jacobian jacb = m_body->get_jacb(tforms);

  // translational x & x_d
  math::Vector3d body_xpos = m_body->get_xpos(tforms);
  math::Vector3d bpos      = m_bpos;
  math::Vector3d bpos_desired
      = xmat_T * (m_frame_desired->get_xpos() - body_xpos);

  math::Vector6d spos = math::Vector6d::Zero();
  spos.head<3>() = math::Quaternion(m_rotm).as_euler(math::EulerAngleSeq::ZYX);
  spos.tail<3>() = bpos;

  math::Vector6d spos_desired = math::Vector6d::Zero();
  spos_desired.head<3>()      = eulr_desired;
  spos_desired.tail<3>()      = bpos_desired;

  // vel eulr angle rates w.r.t. body frame
  math::Vector6d svel
      = T * R_T * m_body->get_xvel_at_bpos(tforms, jacb, qvel, m_bpos);
  math::Vector6d svel_desired = T * R_T * m_frame_desired->get_xvel();

  // math::VectorXd qvel_desired(qvel.size());
  // qvel_desired                  = qvel;
  // qvel_desired[qvel.size() - 6] = eulr_desired[2

  // acc w.r.t. body frame
  math::Vector6d xacc_desired = m_frame_desired->get_xacc();
  // Todo: sacc_desired consists of two extra terms
  math::Vector6d sacc_desired = T * R_T * xacc_desired;

  // go through the targets and resolve the acc
  math::VectorXd sacc_resolved(m_targets.size());

  for (std::size_t i = 0; i < m_targets.size(); i++) {
    if (m_targets[i]->m_type != TargetType::FORCE
        && m_targets[i]->m_type != TargetType::COUPLED) {
      sacc_resolved[i] = m_targets[i]->generate(
          spos[i],
          svel[i],
          spos_desired[i],
          svel_desired[i],
          sacc_desired[i],
          bwrench_sensed[i]);
    }
    else {
      sacc_resolved[i] = 0.0;
    }
  }

  math::Vector6d bacc_resolved = math::Vector6d::Zero();

  bacc_resolved = T.inverse() * sacc_resolved;

  // deal with coupling targets
  for (std::size_t i = 0; i < m_targets.size(); i++) {
    if (m_targets[i]->m_type == TargetType::COUPLED) {
      CoupleTargetPtr ctp
          = std::dynamic_pointer_cast<CoupleTarget>(m_targets[i]);

      CROBOT_ASSERT_ERROR(
          ctp != nullptr,
          "CoupleTarget is nullptr, dynamic_pointer_cast failed.");

      sacc_resolved[i] = ctp->generate(bacc_resolved[ctp->get_leader_id()]);
      bacc_resolved[i] = sacc_resolved[i];
    }
  }

  // std::cout << "spos_error: " << (spos_desired - spos).transpose() <<
  // std::endl;

  // total wrench <bfrc, btorq> needed for a given acc
  math::Vector6d bfrc_btorq = m_body->get_ext_bfrc_btorq_with_bacc(
      tforms, jacb, qvel, bacc_resolved.tail<3>(), bacc_resolved.head<3>());

  math::Vector6d bwrench = bfrc_btorq;
  bwrench.head<3>()      = bfrc_btorq.tail<3>();
  bwrench.tail<3>()      = bfrc_btorq.head<3>();

  for (std::size_t i = 0; i < m_targets.size(); i++) {
    if (m_targets[i]->m_type == TargetType::FORCE) {
      bwrench[i] = m_targets[i]->generate(
          bwrench_sensed[i],
          0.0,
          bwrench_desired[i],
          0.0,
          bwrench_desired[i],
          0.0);
    }
    else {
      bwrench[i] += bwrench_desired[i];
    }
  }

  return std::make_pair(bacc_resolved, bwrench);
}

//==============================================================================
kdl::ConstRigidBodyPtr BodyTarget::get_body() const {
  return m_body;
}
