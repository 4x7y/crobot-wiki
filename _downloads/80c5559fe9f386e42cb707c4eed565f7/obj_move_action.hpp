#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/pose_trajectory.hpp"

namespace crobot::control {

class ObjMoveAction : public Action {
public:
  using Ptr      = std::shared_ptr<ObjMoveAction>;
  using ConstPtr = std::shared_ptr<const ObjMoveAction>;

  struct ContactControlTarget
  {
    std::string body_name;
    math::Vector3d bfrc_desired;
    std::vector<int> dofs_controlled;
  };

  ObjMoveAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::Grasp::ConstPtr grasp,
      const std::vector<ContactControlTarget>& contact_targets,
      const math::Matrix4d& obj_xform_start,
      const math::Matrix4d& obj_xform_end,
      double acceleration);

  ObjMoveAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::Grasp::ConstPtr grasp,
      planning::PoseTrajectory::ConstPtr traj,
      const std::vector<ContactControlTarget>& contact_targets);

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
  kdl::ConstRigidBodyTreePtr m_rbtree;

  kdl::Grasp::ConstPtr m_grasp;

  control::AdmittanceController::Ptr m_controller;

  planning::PoseTrajectory::ConstPtr m_traj;

  std::vector<std::tuple<
      control::AdmittanceControlTarget::Ptr,
      kdl::WorldFramePtr,
      math::Matrix4d>>
      m_ctrl_targets;

  std::vector<int> m_target_qpos_addr;
};

} // namespace crobot::control

