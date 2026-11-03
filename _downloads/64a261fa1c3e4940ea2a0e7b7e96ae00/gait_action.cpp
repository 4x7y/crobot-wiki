#include "crobot/control/gait_action.hpp"

#include "crobot/math/rotation.hpp"
#include "crobot/planning/linear_time_scaling.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
GaitAction::GaitAction(
    std::tuple<double, double, double> duration,
    kdl::ConstRigidBodyTreePtr rbtree,
    GaitAction::ConstPtr gait_prev,
    kdl::Grasp::ConstPtr grasp_target,
    planning::PivotTarget::ConstPtr pivot_target,
    const math::Quaternion& xquat_obj_target)
  : Action(
      "GaitAction",
      std::get<0>(duration) + std::get<1>(duration) + std::get<2>(duration)),
    m_duration(duration),
    m_rbtree(rbtree),
    m_grasp(grasp_target) {
  (void)gait_prev;
  (void)pivot_target;
  (void)xquat_obj_target;
}

//==============================================================================
double GaitAction::get_pivoting_duration() const {
  return std::get<0>(m_duration);
}

//==============================================================================
double GaitAction::get_finger_relocation_duration() const {
  return std::get<1>(m_duration);
}

//==============================================================================
double GaitAction::get_contact_build_duration() const {
  return std::get<2>(m_duration);
}

//==============================================================================
math::Quaternion GaitAction::get_object_xquat() const {
  math::VectorXd qpos_obj = m_pivot_traj->get_position(get_pivoting_duration());
  return math::Quaternion(qpos_obj[3], qpos_obj[4], qpos_obj[5], qpos_obj[6]);
}

//==============================================================================
kdl::Grasp::ConstPtr GaitAction::get_grasp() const {
  return m_grasp;
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> GaitAction::get_command(
    double t, const math::VectorXd& qpos, const math::VectorXd& qvel_actual) {
  (void)qpos;
  (void)qvel_actual;
  if (t <= std::get<0>(m_duration)) {
    return get_pivoting_command(t);
  }
  else if (t <= std::get<0>(m_duration) + std::get<1>(m_duration)) {
    return get_finger_relocation_command(t - std::get<0>(m_duration));
  }
  else {
    return get_contact_build_command(
        t - std::get<0>(m_duration) - std::get<1>(m_duration));
  }
}

//==============================================================================
void GaitAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  (void)qpos_actual;
  (void)qpos_cmd;
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> GaitAction::get_pivoting_command(
    double /* t */) const {
  for (const auto& ctrl_target : m_ctrl_targets_pivoting) {
    ctrl_target->set_desired_force(math::Vector3d(0, 0.0, 0.0));
  }

  return std::make_pair(math::VectorXd(), math::VectorXd());
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd>
GaitAction::get_finger_relocation_command(double t) const {
  (void)t;
  return std::make_pair(math::VectorXd(), math::VectorXd());
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> GaitAction::get_contact_build_command(
    double t) const {
  (void)t;
  return std::make_pair(math::VectorXd(), math::VectorXd());
}
