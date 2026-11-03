#include "crobot/control/simple_joint_move_action.hpp"

#include "crobot/common/logging.hpp"
#include "crobot/planning/trapvel_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
SimpleJointMoveAction::SimpleJointMoveAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::vector<JointControlTarget>& joint_targets,
    double duration)
  : Action("SimpleJointMoveAction", duration), m_rbtree(rbtree) {
  for (const auto& [joint_name, value] : joint_targets) {
    std::pair<int, int> qpos_addr
        = m_rbtree->get_joint(joint_name)->get_qpos_addr()[0];
    CROBOT_ASSERT_ERROR(
        qpos_addr.second == 1, "Only single-DOF joints are supported");
    m_qpos_target.push_back(std::make_pair(qpos_addr.first, value));
  }
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> SimpleJointMoveAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  (void)qpos_actual;
  (void)qvel_actual;

  math::VectorXd qpos_ref      = qpos_cmd;
  math::VectorXd qpos_tracking = m_qpos_traj->get_position(time);
  for (size_t i = 0; i < m_qpos_target.size(); ++i) {
    int qpos_idx       = m_qpos_target[i].first;
    qpos_ref[qpos_idx] = qpos_tracking[i];
  }

  return std::make_pair(qpos_ref, qpos_ref);
}

//==============================================================================
void SimpleJointMoveAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  (void)qpos_actual;
  m_is_inited = true;
  m_is_active = true;
  m_type      = SIMPLE_JMOVE;
  math::VectorXd qpos_start(m_qpos_target.size());
  math::VectorXd qpos_final(m_qpos_target.size());
  for (std::size_t i = 0; i < m_qpos_target.size(); ++i) {
    int addr      = m_qpos_target[i].first;
    qpos_start[i] = qpos_cmd[addr];
    qpos_final[i] = m_qpos_target[i].second;
  }
  m_qpos_traj = std::make_shared<planning::LinearTrajectory>(
      qpos_start,
      qpos_final,
      std::make_shared<planning::TrapvelTimeScaling>(get_duration()));
}

//==============================================================================
void SimpleJointMoveAction::fix_joint(const std::string& joint_name) {
  (void)joint_name;
  CROBOT_LOG_WARN("SimpleJointMoveAction::fix_joint is not implemented");
}

//==============================================================================
void SimpleJointMoveAction::fix_joint(
    const std::string& joint_name, double value) {
  (void)joint_name;
  (void)value;

  CROBOT_LOG_WARN("SimpleJointMoveAction::fix_joint is not implemented");
}

//==============================================================================
void SimpleJointMoveAction::fix_body(const std::string& body_name) {
  (void)body_name;
  CROBOT_LOG_WARN("SimpleJointMoveAction::fix_body is not implemented");
}

//==============================================================================
void SimpleJointMoveAction::set_null_space_obj([
    [maybe_unused]] ik::NullSpaceObj::Ptr obj) {
  // DO NOTHING
}

//==============================================================================
kdl::Grasp::ConstPtr SimpleJointMoveAction::get_grasp() const {
  return nullptr;
}
