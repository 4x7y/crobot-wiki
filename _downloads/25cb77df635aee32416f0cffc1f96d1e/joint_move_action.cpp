#include "crobot/control/joint_move_action.hpp"
#include "crobot/planning/trapvel_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
JointMoveAction::JointMoveAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::Grasp::ConstPtr grasp_target,
    const math::Matrix4d& xform_obj,
    const std::vector<ContactControlTarget>& contact_targets,
    const std::vector<JointControlTarget>& joint_targets,
    double duration)
  : Action("JointMoveAction", duration),
    m_rbtree(rbtree),
    m_grasp(grasp_target),
    m_xform_obj(xform_obj),
    m_contact_targets(contact_targets),
    m_joint_targets(joint_targets) {
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
std::pair<math::VectorXd, math::VectorXd> JointMoveAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  math::VectorXd qpos_ref_actual = qpos_actual;
  math::VectorXd qpos_ref_cmd    = qpos_cmd;
  math::VectorXd qpos_tracking   = m_qpos_traj->get_position(time);
  for (size_t i = 0; i < m_qpos_target.size(); ++i) {
    int qpos_idx              = m_qpos_target[i].first;
    qpos_ref_actual[qpos_idx] = qpos_tracking[i];
    qpos_ref_cmd[qpos_idx]    = qpos_tracking[i];
  }
  if (!m_controller->compute(qpos_ref_actual, qpos_ref_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }
  return m_controller->get_result();
}

//==============================================================================
void JointMoveAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {

  m_is_inited = true;
  m_is_active = true;
  m_type      = JMOVE;

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

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);
  for (std::size_t i = 0; i < m_contact_targets.size(); i++) {
    const auto& target                 = m_contact_targets[i];
    const std::string body_name        = target.body_name;
    kdl::BodyContact::ConstPtr contact = m_grasp->get_contact(body_name);
    CROBOT_ASSERT_ERROR(
        contact != nullptr, "bodyname \"%s\" not found.", body_name.c_str());
    kdl::ConstRigidBodyPtr body   = m_rbtree->get_body(body_name);
    math::Matrix4d tform_ctc2body = contact->get_frame2()->get_tform();
    math::Matrix4d xform_ctc      = body->get_xform(tforms) * tform_ctc2body;

    kdl::WorldFramePtr xframe_desired = std::make_shared<kdl::WorldFrame>();
    xframe_desired->set_xform(xform_ctc);
    AdmittanceControlTarget::Ptr ctrl_target = m_controller->add_target(
        xframe_desired,
        body_name,
        contact->get_frame2()->get_tform().tform2rotm(),
        contact->get_frame2()->get_tform().tform2trvec(),
        math::EulerAngleSeq::ZYX,
        target.dofs_controlled,
        contact);
    ctrl_target->set_desired_force(m_contact_targets[i].bfrc_desired);
    m_ctrl_targets.push_back(std::make_pair(ctrl_target, xframe_desired));
  }
}

//==============================================================================
void JointMoveAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void JointMoveAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void JointMoveAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}
//==============================================================================
kdl::Grasp::ConstPtr JointMoveAction::get_grasp() const {
  return m_grasp;
}

//==============================================================================
void JointMoveAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
