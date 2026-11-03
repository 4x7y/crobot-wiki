#pragma once

#include <memory>

#include "crobot/ik/null_space_objective.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class Action {
public:
  using Ptr      = std::shared_ptr<Action>;
  using ConstPtr = std::shared_ptr<const Action>;

  enum ActionType
  {
    NONE,
    PIVOT,
    JMOVE,
    SMOOTH,
    RELOCATION,
    FORCE_ADJUST,
    SIMPLE_JMOVE,
    ROLLING,
    OBJ_MOVE,
    // SLIDING,
  };

  Action(const std::string& name, double duration);

  virtual ~Action() = default;

  virtual double get_duration() const;

  virtual std::pair<math::VectorXd, math::VectorXd> get_command(
      double time,
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel_actual = {})
      = 0;

  /**
   * @brief Initialization of the action. This function will be called at the
   * begining of the action.
   *
   * @param qpos_actual The initial actual robot configuration
   * @param qpos_cmd The initial robot configuration command
   */
  virtual void initialize(
      const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd)
      = 0;

  void set_name(const std::string& name);

  bool is_inited() const;

  bool is_active() const;

  ActionType get_type() const;

  const std::string& get_name() const;

  void set_dt(double dt);

  virtual void fix_joint(const std::string& joint_name) = 0;

  virtual void fix_joint(const std::string& joint_name, double value) = 0;

  virtual void fix_body(const std::string& body_name) = 0;

  virtual kdl::Grasp::ConstPtr get_grasp() const = 0;

  virtual void set_null_space_obj(ik::NullSpaceObj::Ptr null_space_obj) = 0;

protected:
  bool m_is_inited  = false;
  ActionType m_type = NONE;
  double m_dt       = 0.002;
  bool m_is_active  = false;

private:
  double m_duration;
  std::string m_name;
};

} // namespace crobot::control

