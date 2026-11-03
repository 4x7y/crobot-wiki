#include "crobot/control/force_adjustment_action.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/planning/linear_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
ForceAdjustmentAction::ForceAdjustmentAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::ConstRigidBodyPtr body,
    kdl::Grasp::ConstPtr grasp_prev,
    kdl::Grasp::ConstPtr grasp,
    const std::vector<ForceAdjTarget>& targets,
    double duration)
  : Action("ForceAdjustmentAction", duration),
    m_rbtree(rbtree),
    m_body(body),
    m_grasp_prev(grasp_prev),
    m_grasp(grasp),
    m_targets(targets) {
  // CROBOT_ASSERT_ERROR(
  //     grasp->get_num_contacts() == static_cast<int>(targets.size()));
  m_controller = std::make_shared<control::AdmittanceController>(rbtree);
  m_force_trajs.reserve(grasp->get_num_contacts());
  if (m_targets[0].body_name == m_body->get_name())
    m_body_efrc_defined = true;
  else
    m_body_efrc_defined = false;
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> ForceAdjustmentAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  std::size_t start_id = 0;
  math::VectorXd cwrench_resolved
      = math::VectorXd::Zero(m_grasp_prev->get_num_wrench_basis());
  // if the body wrench is defined, then we need to compute the external
  // wrench
  if (m_body_efrc_defined) {
    math::Vector3d xfrc             = m_force_trajs[0]->get_position(time);
    math::Vector6d bwrench_resolved = math::Vector6d::Zero();
    bwrench_resolved.head<3>() = m_xform_obj.tform2rotm().transpose() * xfrc;
    cwrench_resolved           = m_mat_grasp_pinv * bwrench_resolved;
    start_id                   = 1;
  }

  for (std::size_t i = 0; i < m_ctrl_targets.size(); i++) {
    std::size_t id_ctc = m_name_to_index[m_targets[i].body_name];
    m_cfrc_internal_stack.segment(id_ctc * 3, 3)
        = m_force_trajs[i + start_id]->get_position(time);
  }

  math::VectorXd cfrc_internal
      = m_mat_null_space_mapping * m_cfrc_internal_stack;

  for (std::size_t i = 0; i < m_ctrl_targets.size(); ++i) {
    AdmittanceControlTarget::Ptr ctrl_target = m_ctrl_targets[i];
    std::string body_name                    = ctrl_target->get_name(2);

    kdl::BodyContact::Ptr contact = m_grasp->get_contact(body_name);

    math::Vector3d cfrc_i
        = cfrc_internal.segment(m_name_to_index[body_name] * 3, 3);
    math::Vector3d cfrc
        = cwrench_resolved.segment(m_name_to_index_prev[body_name] * 3, 3);
    ctrl_target->set_desired_force(cfrc + cfrc_i);
  }
  if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  return m_controller->get_result();
}

//==============================================================================
void ForceAdjustmentAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {

  (void)qpos_cmd;
  m_is_inited = true;
  m_is_active = true;
  m_type      = FORCE_ADJUST;

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  m_xform_obj = m_body->get_xform(tforms);

  m_mat_grasp_pinv_prev
      = math::calculate_jacb_inv(m_grasp_prev->get_grasp_matrix());

  m_num_wrench_biasis      = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp = m_grasp->get_grasp_matrix();
  m_mat_grasp_pinv         = math::calculate_jacb_inv(mat_grasp);
  m_mat_null_space_mapping
      = math::MatrixXd::Identity(m_num_wrench_biasis, m_num_wrench_biasis)
        - m_mat_grasp_pinv * mat_grasp;

  std::vector<kdl::BodyContact::Ptr> contacts = m_grasp->get_contacts();
  m_cfrc_internal_stack = math::VectorXd::Zero(m_num_wrench_biasis);

  for (std::size_t k = 0; k < contacts.size(); k++) {
    kdl::BodyContact::Ptr contact = contacts[k];
    for (std::size_t i = 0; i < m_targets.size(); i++) {
      if (contact->get_body2()->get_name() == m_targets[i].body_name) {
        m_cfrc_internal_stack.segment(k * 3, 3) = m_targets[i].force_start;
        m_name_to_index[contact->get_body2()->get_name()] = k;
        break;
      }
    }
  }

  std::vector<kdl::BodyContact::Ptr> contacts_prev
      = m_grasp_prev->get_contacts();

  for (std::size_t k = 0; k < contacts_prev.size(); k++) {
    kdl::BodyContact::Ptr contact = contacts_prev[k];
    for (std::size_t i = 0; i < m_targets.size(); i++) {
      if (contact->get_body2()->get_name() == m_targets[i].body_name) {
        m_name_to_index_prev[contact->get_body2()->get_name()] = k;
        break;
      }
    }
  }

  for (std::size_t i = 0; i < m_targets.size(); i++) {
    const std::string body_name = m_targets[i].body_name;
    if (body_name != m_body->get_name()) {
      kdl::WorldFrame::Ptr xframe_desired = std::make_shared<kdl::WorldFrame>();
      kdl::BodyContact::ConstPtr contact  = m_grasp->get_contact(body_name);
      CROBOT_ASSERT_ERROR(
          contact != nullptr, "bodyname \"%s\" not found.", body_name.c_str());
      control::AdmittanceControlTarget::Ptr ctrl_target
          = m_controller->add_target(
              xframe_desired,
              body_name,
              contact->get_frame2()->get_tform().tform2rotm(),
              contact->get_frame2()->get_tform().tform2trvec(),
              math::EulerAngleSeq::ZYX,
              m_targets[i].dofs_controlled,
              contact);

      math::Matrix4d xform_obj = m_body->get_xform(tforms);
      xframe_desired->set_xform(xform_obj * contact->get_frame1()->get_tform());

      CROBOT_ASSERT_ERROR(ctrl_target != nullptr, "Invalid ControlTarget");
      m_ctrl_targets.push_back(ctrl_target);
    }

    m_force_trajs.push_back(std::make_shared<planning::LinearTrajectory>(
        m_targets[i].force_start,
        m_targets[i].force_final,
        std::make_shared<planning::LinearTimeScaling>(get_duration())));
  }
}

//==============================================================================
void ForceAdjustmentAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void ForceAdjustmentAction::fix_joint(
    const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void ForceAdjustmentAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}

//==============================================================================
kdl::Grasp::ConstPtr ForceAdjustmentAction::get_grasp() const {
  return m_grasp_prev;
}

//==============================================================================
void ForceAdjustmentAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
