#pragma once

#include "crobot/control/action.hpp"

namespace crobot::control {

class SmoothAction : public Action {
public:
  SmoothAction(
      const std::string& name,
      double duration,
      Action::Ptr action_prev,
      Action::Ptr action_next);

  SmoothAction(
      const std::string& name,
      double duration,
      math::VectorXd qpos,
      Action::Ptr action_next);

  std::pair<math::VectorXd, math::VectorXd> get_command(
      double time,
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel_actual = {}) override;

  void initialize(
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd) override;

  void fix_joint(const std::string& joint_name);

  void fix_joint(const std::string& joint_name, double value);

  void fix_body(const std::string& body_name);

  kdl::Grasp::ConstPtr get_grasp() const override;

  void set_null_space_obj(ik::NullSpaceObj::Ptr obj) override;

private:
  double m_factor;
  Action::Ptr m_action_prev;
  Action::Ptr m_action_next;

  // initialize
  math::VectorXd m_qpos_actual_prev;
  // math::VectorXd m_qpos_actual_next;
  math::VectorXd m_qpos_command_prev;
  // math::VectorXd m_qpos_command_next;
};

} // namespace crobot::control

