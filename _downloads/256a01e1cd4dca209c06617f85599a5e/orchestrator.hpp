#pragma once

#include "crobot/control/action.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class Orchestrator {
public:
  Orchestrator(
      double time_step,
      const math::VectorXd& qpos_init,
      const math::VectorXd& qvel_init);

  math::VectorXd next(
      const math::VectorXd& qpos_actual = math::VectorXd::Zero(0),
      const math::VectorXd& qvel_actual = math::VectorXd::Zero(0));

  void add_action(Action::Ptr action, double smooth_time = 1.0);

  double get_duration() const;

  double get_time() const;

  bool has_next() const;

  math::VectorXd get_qpos() const;

  void reset();

  kdl::Grasp::ConstPtr get_grasp() const;

  void set_obj_pose(int qpos_start, const math::VectorXd& qpos_obj);

private:
  double m_time;
  double m_time_step;
  double m_duration;

  math::VectorXd m_qpos_actual;
  math::VectorXd m_qpos_cmd;

  math::VectorXd m_qvel;

  std::vector<Action::Ptr> m_actions;

  kdl::Grasp::ConstPtr m_grasp_active;
};

} // namespace crobot::control

