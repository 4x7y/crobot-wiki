#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/control/body_target.hpp"
#include "crobot/control/obj_xpos_ctrl_target.hpp"
#include "crobot/kdl/dynamic_grasp.hpp"
#include "crobot/planning/pose_trajectory.hpp"
#include "crobot/planning/screwdriver_trajectory.hpp"

namespace crobot::control {

class RollingAction2 : public Action {
public:
  using Ptr      = std::shared_ptr<RollingAction2>;
  using ConstPtr = std::shared_ptr<const RollingAction2>;

  struct ContactTarget
  {
    std::string body_name;
    std::vector<int> dofs_controlled;
    math::Vector3d bfrc_desired;
  };

  RollingAction2(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::ConstRigidBodyPtr body,
      kdl::DynamicGrasp::ConstPtr grasp_target,
      const planning::PoseTrajectory::ConstPtr& body_traj,
      const math::Vector6d& xwrench_ext_desired,
      const std::vector<ContactTarget>& contact_targets);

  std::pair<math::VectorXd, math::VectorXd> get_command(
      double time,
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel_actual) override;

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
  kdl::ConstRigidBodyPtr m_body;
  kdl::DynamicGrasp::ConstPtr m_grasp;
  planning::PoseTrajectory::ConstPtr m_body_traj;
  math::Vector6d m_xwrench_ext_desired;

  std::vector<ContactTarget> m_contact_targets;

  // kdl::WorldFramePtr m_body_target_desired_xframe;
  // control::BodyTargetPtr m_body_target;

  control::AdmittanceController::Ptr m_controller;

  ObjXposCtrlTargetPtr m_obj_ctrl;
  kdl::WorldFramePtr m_obj_ctrl_desired_xframe;

  std::vector<
      std::pair<control::AdmittanceControlTarget::Ptr, kdl::WorldFramePtr>>
      m_ctrl_targets;

  std::vector<int> m_target_qpos_addr;
  std::vector<int> m_target_qvel_addr;

  math::VectorXd m_cfrc_internal_stack;

  // body name to the contact index in grasp
  std::unordered_map<std::string, std::size_t> m_name_to_index;
};

} // namespace crobot::control

