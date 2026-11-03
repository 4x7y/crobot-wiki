#include "crobot/control/obj_move_action.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/planning/planning.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
ObjMoveAction::ObjMoveAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::Grasp::ConstPtr grasp,
    const std::vector<ContactControlTarget>& contact_targets,
    const math::Matrix4d& obj_xform_start,
    const math::Matrix4d& obj_xform_end,
    double acceleration)
  : Action("ObjMoveAction", 2 * std::sqrt(1.0 / acceleration)), m_grasp(grasp) {
  m_traj = std::make_shared<planning::PoseTrajectory>(
      obj_xform_start,
      obj_xform_end,
      std::make_shared<planning::ConstaccTimeScaling>(acceleration));

  m_controller = std::make_shared<control::AdmittanceController>(rbtree);

  for (const ContactControlTarget& contact_target : contact_targets) {
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
    ctrl_target->set_desired_force(contact_target.bfrc_desired);
    m_ctrl_targets.push_back(std::make_tuple(
        ctrl_target, xframe_desired, contact->get_frame1()->get_tform()));
  }

  m_target_qpos_addr = grasp->get_body()->get_support_qpos_addr();
}

//==============================================================================
ObjMoveAction::ObjMoveAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::Grasp::ConstPtr grasp,
    planning::PoseTrajectory::ConstPtr traj,
    const std::vector<ContactControlTarget>& contact_targets)
  : Action("ObjMoveAction", traj->get_duration()),
    m_grasp(grasp),
    m_traj(traj) {

  m_controller = std::make_shared<control::AdmittanceController>(rbtree);

  for (const ContactControlTarget& contact_target : contact_targets) {
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
    ctrl_target->set_desired_force(contact_target.bfrc_desired);
    m_ctrl_targets.push_back(std::make_tuple(
        ctrl_target, xframe_desired, contact->get_frame1()->get_tform()));
  }

  m_target_qpos_addr = grasp->get_body()->get_support_qpos_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> ObjMoveAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  // xpos, xquat
  math::VectorXd qpos_obj = m_traj->get_position(time);
  math::Matrix4d xform_obj
      = math::Quaternion(qpos_obj(3), qpos_obj(4), qpos_obj(5), qpos_obj(6))
            .as_tform();
  xform_obj.block<3, 1>(0, 3) = qpos_obj.head<3>();

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
void ObjMoveAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  CROBOT_LOG_INFO("Initializing ObjMoveAction: {}", get_name());
  CROBOT_LOG_WARN("Do nothing in ObjMoveAction::initialize()");
  (void)qpos_actual;
  (void)qpos_cmd;
  m_is_inited = true;
  m_is_active = true;
  m_type      = PIVOT;
  CROBOT_LOG_INFO("Completed initializing ObjMoveAction: {}", get_name());
}

//==============================================================================
void ObjMoveAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void ObjMoveAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void ObjMoveAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}
//==============================================================================
kdl::Grasp::ConstPtr ObjMoveAction::get_grasp() const {
  return m_grasp;
}

//==============================================================================
void ObjMoveAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
