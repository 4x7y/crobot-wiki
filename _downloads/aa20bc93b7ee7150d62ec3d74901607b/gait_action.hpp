#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/pivot_target.hpp"
#include "crobot/planning/pivot_trajectory.hpp"

namespace crobot::control {

class GaitAction : public Action {
public:
  using Ptr      = std::shared_ptr<GaitAction>;
  using ConstPtr = std::shared_ptr<const GaitAction>;

  GaitAction(
      std::tuple<double, double, double> duration,
      kdl::ConstRigidBodyTreePtr rbtree,
      GaitAction::ConstPtr gait_prev,
      kdl::Grasp::ConstPtr grasp_target,
      planning::PivotTarget::ConstPtr pivot_target,
      const math::Quaternion& xquat_obj_target);

  virtual ~GaitAction() = default;

  virtual double get_pivoting_duration() const;

  virtual double get_finger_relocation_duration() const;

  virtual double get_contact_build_duration() const;

  virtual math::Quaternion get_object_xquat() const;

  virtual kdl::Grasp::ConstPtr get_grasp() const;

  virtual std::pair<math::VectorXd, math::VectorXd> get_command(
      double t,
      const math::VectorXd& qpos,
      const math::VectorXd& qvel_actual = {});

  void initialize(
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd) override;

protected:
  virtual std::pair<math::VectorXd, math::VectorXd> get_pivoting_command(
      double t) const;

  virtual std::pair<math::VectorXd, math::VectorXd>
  get_finger_relocation_command(double t) const;

  virtual std::pair<math::VectorXd, math::VectorXd> get_contact_build_command(
      double t) const;

private:
  std::tuple<double, double, double> m_duration;
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::Grasp::ConstPtr m_grasp;
  control::AdmittanceController::Ptr m_controller_pivoting;
  planning::PivotTrajectory::ConstPtr m_pivot_traj;
  std::vector<control::AdmittanceControlTarget::Ptr> m_ctrl_targets_pivoting;
};

} // namespace crobot::control

