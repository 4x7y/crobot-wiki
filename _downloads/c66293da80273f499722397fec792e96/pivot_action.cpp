#include "crobot/control/pivot_action.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/planning/trapvel_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
PivotAction::PivotAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    planning::PivotTarget::ConstPtr pivot_target,
    const math::Quaternion& xquat_start,
    const math::Quaternion& xquat_end,
    kdl::Grasp::ConstPtr grasp,
    const std::vector<ContactControlTarget>& contact_targets,
    double duration)
  : Action("PivotAction", duration), m_object(pivot_target), m_grasp(grasp) {
  m_traj = std::make_shared<planning::PivotTrajectory>(
      pivot_target,
      xquat_start,
      xquat_end,
      std::make_shared<planning::TrapvelTimeScaling>(duration));

  m_controller = std::make_shared<control::AdmittanceController>(rbtree);

  int num_wrench_biasis         = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp      = m_grasp->get_grasp_matrix();
  math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;

  math::VectorXd cfrc_internal_stack(3 * contact_targets.size());
  for (size_t i = 0; i < contact_targets.size(); i++) {
    cfrc_internal_stack.segment<3>(3 * i) = contact_targets[i].bfrc_desired;
  }

  math::VectorXd cfrc_internal = mat_null_space_mapping * cfrc_internal_stack;

  for (size_t i = 0; i < contact_targets.size(); i++) {
    const ContactControlTarget& contact_target = contact_targets[i];
    kdl::BodyContact::ConstPtr contact
        = m_grasp->get_contact(contact_target.body_name);
    kdl::WorldFramePtr xframe_desired = std::make_shared<kdl::WorldFrame>();
    control::AdmittanceControlTarget::Ptr ctrl_target
        = m_controller->add_target(
            xframe_desired,
            contact->get_body2()->get_name(),
            contact->get_frame2()->get_tform().tform2rotm(),
            contact->get_frame2()->get_tform().tform2trvec(),
            math::EulerAngleSeq::ZYX,
            contact_target.dofs_controlled,
            contact);
    ctrl_target->set_desired_force(cfrc_internal.segment<3>(3 * i));
    m_ctrl_targets.push_back(std::make_tuple(
        ctrl_target, xframe_desired, contact->get_frame1()->get_tform()));
  }

  m_target_qpos_addr = rbtree->get_body(m_object->get_body()->get_name())
                           ->get_support_qpos_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> PivotAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  // xpos, xquat
  math::VectorXd qpos_obj = m_traj->get_position(time);
  CROBOT_LOG_DEBUG("PivotAction qpos_obj: {}", qpos_obj.transpose());
  std::vector<math::Matrix4d> tforms
      = m_object->get_rbtree()->forward_kinematics(qpos_obj);
  math::Matrix4d xform_obj = m_object->get_body()->get_xform(tforms);

  for (size_t i = 0; i < m_ctrl_targets.size(); ++i) {
    control::AdmittanceControlTarget::Ptr ctrl_target;
    kdl::WorldFramePtr xframe_desired;
    math::Matrix4d tform_obj;
    std::tie(ctrl_target, xframe_desired, tform_obj) = m_ctrl_targets[i];
    xframe_desired->set_xform(xform_obj * tform_obj);
  }

  // update target pose for visualization
  math::VectorXd qpos_actual_ret, qpos_cmd_ret;

  if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  std::tie(qpos_actual_ret, qpos_cmd_ret) = m_controller->get_result();

  qpos_actual_ret(m_target_qpos_addr) = qpos_obj;
  qpos_cmd_ret(m_target_qpos_addr)    = qpos_obj;
  return std::make_pair(qpos_actual_ret, qpos_cmd_ret);
}

//==============================================================================
void PivotAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  CROBOT_LOG_INFO("Initializing PivotAction: {}", get_name());
  CROBOT_LOG_WARN("Do nothing in PivotAction::initialize()");
  (void)qpos_actual;
  (void)qpos_cmd;
  m_is_inited = true;
  m_is_active = true;
  m_type      = PIVOT;
  CROBOT_LOG_INFO("Completed initializing PivotAction: {}", get_name());
}

//==============================================================================
void PivotAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void PivotAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void PivotAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}
//==============================================================================
kdl::Grasp::ConstPtr PivotAction::get_grasp() const {
  return m_grasp;
}

//==============================================================================
void PivotAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
