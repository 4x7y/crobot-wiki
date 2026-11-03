#include "crobot/control/body_move_action.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/planning/planning.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
BodyMoveAction::BodyMoveAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name,
    const math::Matrix4d& body_tform,
    planning::Trajectory::ConstPtr traj,
    std::vector<int> dofs_controlled)
  : Action("BodyMoveAction", traj->get_duration()),
    m_rbtree(rbtree),
    m_body(rbtree->get_body(body_name)),
    m_traj(traj) {

  m_controller = std::make_shared<control::AdmittanceController>(rbtree);

  m_xframe_desired = std::make_shared<kdl::WorldFrame>();
  m_ctrl_target    = m_controller->add_target(
      m_xframe_desired,
      body_name,
      body_tform.tform2rotm(),
      body_tform.tform2trvec(),
      math::EulerAngleSeq::ZYX,
      dofs_controlled);

  m_target_qpos_addr = m_body->get_support_qpos_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> BodyMoveAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  // xpos, xquat
  math::VectorXd qpos_body = m_traj->get_position(time);
  math::Matrix4d xform_body
      = math::Quaternion(qpos_body(3), qpos_body(4), qpos_body(5), qpos_body(6))
            .as_tform();
  xform_body.block<3, 1>(0, 3) = qpos_body.head<3>();
  m_xframe_desired->set_xform(xform_body);

  // update target pose for visualization
  math::VectorXd qpos_actual_ret, qpos_cmd_ret;

  if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  return m_controller->get_result();
}

//==============================================================================
void BodyMoveAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  CROBOT_LOG_INFO("Initializing BodyMoveAction: {}", get_name());
  (void)qpos_actual;
  (void)qpos_cmd;
  m_is_inited = true;
  m_is_active = true;
  m_type      = PIVOT;
  CROBOT_LOG_INFO("Completed initializing BodyMoveAction: {}", get_name());
}

//==============================================================================
void BodyMoveAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void BodyMoveAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void BodyMoveAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}

//==============================================================================
void BodyMoveAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}

//==============================================================================
kdl::Grasp::ConstPtr BodyMoveAction::get_grasp() const {
  // CROBOT_LOG_WARN("Do nothing in BodyMoveAction::get_grasp()");
  return nullptr;
}
