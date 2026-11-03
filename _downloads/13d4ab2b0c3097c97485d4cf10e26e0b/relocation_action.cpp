#include "crobot/control/relocation_action.hpp"

#include "crobot/common/logging.hpp"
#include "crobot/planning/pose_trajectory.hpp"
#include "crobot/planning/trapvel_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
RelocationAction::RelocationAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::ConstRigidBodyPtr body,
    kdl::DynamicGrasp::ConstPtr grasp_prev,
    kdl::DynamicGrasp::ConstPtr grasp_target,
    const math::Vector6d& xwrench_ext_desired,
    const std::vector<ActionTarget>& relocate_targets,
    const std::vector<JointControlTarget>& joint_targets,
    const std::vector<ContactTarget>& contact_targets,
    double duration)
  : Action("RelocationAction", duration),
    m_rbtree(rbtree),
    m_body(body),
    m_grasp_prev(grasp_prev),
    m_grasp(grasp_target),
    m_xwrench_ext_desired(xwrench_ext_desired),
    m_relocate_targets(relocate_targets),
    m_joint_targets(joint_targets),
    m_contact_targets(contact_targets) {
  m_controller = std::make_shared<control::AdmittanceController>(rbtree);

  for (const auto& [joint_name, value, action_type] : joint_targets) {
    (void)action_type;
    m_controller->fix_joint(joint_name);
    std::pair<int, int> qpos_addr
        = m_rbtree->get_joint(joint_name)->get_qpos_addr()[0];
    CROBOT_ASSERT_ERROR(
        qpos_addr.second == 1, "Only single-DOF joints are supported");
    m_qpos_target.push_back(std::make_pair(qpos_addr.first, value));
  }
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> RelocationAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  CROBOT_ASSERT_ERROR(
      m_tform_trajs.size() != 0,
      "Relocating manipulator pose trajectory not initialized.");

  // setup joint move targets
  math::VectorXd qpos_ref_actual = qpos_actual;
  math::VectorXd qpos_ref_cmd    = qpos_cmd;
  math::VectorXd qpos_tracking   = m_qpos_traj->get_position(time);
  for (size_t i = 0; i < m_qpos_target.size(); ++i) {
    int qpos_idx              = m_qpos_target[i].first;
    qpos_ref_actual[qpos_idx] = qpos_tracking[i];
    qpos_ref_cmd[qpos_idx]    = qpos_tracking[i];
  }

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);
  math::Matrix3d xmat_o = m_body->get_xmat(tforms);

  // update contacts
  auto contacts = m_grasp_prev->get_dynamic_contacts();
  for (std::size_t i = 0; i < m_contact_targets.size(); ++i) {
    if (m_contact_targets[i].dofs_controlled.size() <= 4) {
      std::size_t contact_id_grasp
          = m_name_to_index[m_contact_targets[i].body_name];
      auto ctc_dir = contacts[contact_id_grasp]->get_rolling_moving_dir(
          tforms, qvel_actual);
      // std::cout << "ctc_dir1: " << ctc_dir.first.transpose() << std::endl;
      contacts[contact_id_grasp]->update_contact_points_in_dir(
          ctc_dir.first, ctc_dir.second, m_dt);
    }
  }

  int num_wrench_biasis         = m_grasp_prev->get_num_wrench_basis();
  math::Matrix6X mat_grasp      = m_grasp_prev->get_grasp_matrix();
  math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;
  math::VectorXd cfrc_internal = mat_null_space_mapping * m_cfrc_internal_stack;

  math::Vector6d bwrench_resolved = math::Vector6d::Zero();
  bwrench_resolved.head<3>()
      = xmat_o.transpose() * m_xwrench_ext_desired.tail<3>();
  bwrench_resolved.tail<3>()
      = xmat_o.transpose() * m_xwrench_ext_desired.head<3>();

  math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;

  for (size_t i = 0; i < m_ctrl_targets.size(); ++i) {
    control::AdmittanceControlTarget::Ptr ctrl_target;
    kdl::WorldFramePtr xframe_desired;
    std::tie(ctrl_target, xframe_desired) = m_ctrl_targets[i];

    // relocating manipulator
    if (i < m_relocate_targets.size()) {
      math::Matrix4d xform_relo
          = m_xform_obj * m_tform_trajs[i]->get_tform(time);
      xframe_desired->set_xform(xform_relo);
      ctrl_target->set_desired_force(math::Vector3d::Zero());
    }
    // other manipulators
    else {
      kdl::BodyRollingContact::Ptr contact = m_grasp_prev->get_dynamic_contact(
          m_contact_targets[i - m_relocate_targets.size()].body_name);
      xframe_desired->set_xform(
          m_xform_obj * contact->get_frame1()->get_tform());

      std::size_t contact_id_grasp
          = m_name_to_index[m_contact_targets[i - m_relocate_targets.size()]
                                .body_name];
      ctrl_target->set_desired_force(
          cfrc_internal.segment(contact_id_grasp * 3, 3)
          + cwrench_resolved.segment(contact_id_grasp * 3, 3));

      // update contact points if 0,1,2 dof not controlled
      // if (m_contact_targets[i - m_relocate_targets.size()]
      //         .dofs_controlled.size()
      //     <= 4) {
      //   auto ctc_dir = contact->get_rolling_moving_dir(tforms, qvel_actual);
      //   contact->update_contact_points_in_dir(
      //       ctc_dir.first, ctc_dir.second, m_dt);
      // }
    }
  }

  if (!m_controller->compute(qpos_ref_actual, qpos_ref_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_ref_actual, qpos_ref_cmd);
  }

  return m_controller->get_result();
}

//==============================================================================
void RelocationAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  (void)qpos_cmd;
  m_is_inited = true;
  m_is_active = true;
  m_type      = RELOCATION;

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  m_xform_obj = m_body->get_xform(tforms);

  for (std::size_t i = 0; i < m_relocate_targets.size(); i++) {
    kdl::ConstRigidBodyPtr body
        = m_rbtree->get_body(m_relocate_targets[i].body_name);
    math::Matrix4d tform_ctc2body
        = m_grasp->get_contact(body->get_name())->get_frame2()->get_tform();
    math::Matrix4d xform_ctc     = body->get_xform(tforms) * tform_ctc2body;
    math::Matrix4d tform_ctc2obj = m_xform_obj.tform_inv() * xform_ctc;

    m_tform_trajs.push_back(std::make_shared<planning::PoseTrajectory>(
        tform_ctc2obj,
        m_grasp->get_contact(body->get_name())->get_frame1()->get_tform(),
        std::make_shared<planning::TrapvelTimeScaling>(get_duration())));
  }

  std::vector<kdl::BodyContact::Ptr> contacts = m_grasp_prev->get_contacts();
  m_cfrc_internal_stack = math::VectorXd::Zero(3 * contacts.size());

  for (std::size_t k = 0; k < contacts.size(); k++) {
    kdl::BodyContact::Ptr contact = contacts[k];
    for (std::size_t i = 0; i < m_contact_targets.size(); i++) {
      if (contact->get_body2()->get_name() == m_contact_targets[i].body_name) {
        m_cfrc_internal_stack.segment(k * 3, 3)
            = m_contact_targets[i].bfrc_desired;
        m_name_to_index[contact->get_body2()->get_name()] = k;
        break;
      }
    }
  }

  CROBOT_LOG_INFO("Set pose trajectory");

  // add relocating manipulator control target
  for (const auto& contact : m_grasp->get_contacts()) {
    for (std::size_t i = 0; i < m_relocate_targets.size(); i++) {
      if (contact->get_body2()->get_name() == m_relocate_targets[i].body_name) {
        kdl::WorldFrame::Ptr xframe_desired
            = std::make_shared<kdl::WorldFrame>();
        control::AdmittanceControlTarget::Ptr ctrl_target
            = m_controller->add_target(
                xframe_desired,
                contact->get_body2()->get_name(),
                contact->get_frame2()->get_tform().tform2rotm(),
                contact->get_frame2()->get_tform().tform2trvec(),
                math::EulerAngleSeq::ZYX,
                m_relocate_targets[i].dofs_controlled,
                contact);
        m_ctrl_targets.push_back(std::make_pair(ctrl_target, xframe_desired));
      }
    }
  }
  CROBOT_ASSERT_ERROR(
      m_ctrl_targets.size() == m_relocate_targets.size(),
      "Relocating body not found in the grasp.");

  CROBOT_LOG_INFO("Set relocating manipulator control target");

  for (std::size_t i = 0; i < m_contact_targets.size(); i++) {
    const std::string body_name = m_contact_targets[i].body_name;
    kdl::BodyRollingContact::ConstPtr contact
        = m_grasp->get_dynamic_contact(body_name);

    kdl::WorldFrame::Ptr xframe_desired = std::make_shared<kdl::WorldFrame>();
    control::AdmittanceControlTarget::Ptr ctrl_target
        = m_controller->add_rolling_target(
            xframe_desired,
            contact->get_body2()->get_name(),
            math::EulerAngleSeq::ZYX,
            m_contact_targets[i].dofs_controlled,
            contact);
    ctrl_target->set_desired_force(m_contact_targets[i].bfrc_desired);

    xframe_desired->set_xform(m_xform_obj * contact->get_frame1()->get_tform());

    m_ctrl_targets.push_back(std::make_pair(ctrl_target, xframe_desired));
  }

  // set joint move target
  math::VectorXd qpos_start(m_qpos_target.size());
  math::VectorXd qpos_final(m_qpos_target.size());
  for (std::size_t i = 0; i < m_qpos_target.size(); ++i) {
    int addr      = m_qpos_target[i].first;
    qpos_start[i] = qpos_cmd[addr];
    if (m_joint_targets[i].action_type == JointControlTarget::SCALE)
      qpos_final[i] = qpos_start[i] * m_qpos_target[i].second;
    else if (m_joint_targets[i].action_type == JointControlTarget::OFFSET)
      qpos_final[i] = qpos_start[i] + m_qpos_target[i].second;
    else
      qpos_final[i] = m_qpos_target[i].second;
  }
  m_qpos_traj = std::make_shared<planning::LinearTrajectory>(
      qpos_start,
      qpos_final,
      std::make_shared<planning::TrapvelTimeScaling>(get_duration()));
}

//==============================================================================
void RelocationAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void RelocationAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void RelocationAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}

//==============================================================================
kdl::Grasp::ConstPtr RelocationAction::get_grasp() const {
  return m_grasp_prev;
}

//==============================================================================
void RelocationAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
