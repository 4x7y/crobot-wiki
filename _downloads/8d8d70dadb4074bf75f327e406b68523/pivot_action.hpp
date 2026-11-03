#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/pivot_trajectory.hpp"

namespace crobot::control {

class PivotAction : public Action {
public:
  using Ptr      = std::shared_ptr<PivotAction>;
  using ConstPtr = std::shared_ptr<const PivotAction>;

  struct ContactControlTarget
  {
    std::string body_name;
    std::vector<int> dofs_controlled;
    math::Vector3d bfrc_desired;
  };

  PivotAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      planning::PivotTarget::ConstPtr pivot_target,
      const math::Quaternion& xquat_start,
      const math::Quaternion& xquat_end,
      kdl::Grasp::ConstPtr grasp,
      const std::vector<ContactControlTarget>& contact_targets,
      double duration);

  // Documentation inherited
  std::pair<math::VectorXd, math::VectorXd> get_command(
      double time,
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel_actual = {}) override;

  // Documentation inherited
  void initialize(
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd) override;

  void fix_joint(const std::string& joint_name) override;

  void fix_joint(const std::string& joint_name, double value) override;

  void fix_body(const std::string& body_name) override;

  kdl::Grasp::ConstPtr get_grasp() const override;

  void set_null_space_obj(ik::NullSpaceObj::Ptr obj) override;

private:
  planning::PivotTarget::ConstPtr m_object;
  kdl::Grasp::ConstPtr m_grasp;
  control::AdmittanceController::Ptr m_controller;
  planning::PivotTrajectory::ConstPtr m_traj;
  std::vector<std::tuple<
      control::AdmittanceControlTarget::Ptr,
      kdl::WorldFramePtr,
      math::Matrix4d>>
      m_ctrl_targets;
  std::vector<int> m_target_qpos_addr;
};

} // namespace crobot::control

