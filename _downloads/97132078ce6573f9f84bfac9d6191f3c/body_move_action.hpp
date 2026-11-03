#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/planning.hpp"

namespace crobot::control {

class BodyMoveAction : public Action {
public:
  using Ptr      = std::shared_ptr<BodyMoveAction>;
  using ConstPtr = std::shared_ptr<const BodyMoveAction>;

  BodyMoveAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_name,
      const math::Matrix4d& body_tform,
      planning::Trajectory::ConstPtr traj,
      std::vector<int> dofs_controlled);

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

  void set_null_space_obj(ik::NullSpaceObj::Ptr obj) override;

  kdl::Grasp::ConstPtr get_grasp() const;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;

  control::AdmittanceController::Ptr m_controller;

  control::AdmittanceControlTarget::Ptr m_ctrl_target;

  kdl::WorldFramePtr m_xframe_desired;

  planning::Trajectory::ConstPtr m_traj;

  std::vector<int> m_target_qpos_addr;
};

} // namespace crobot::control

