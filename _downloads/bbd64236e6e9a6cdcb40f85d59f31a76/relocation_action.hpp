#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/dynamic_grasp.hpp"
#include "crobot/planning/pose_trajectory.hpp"

namespace crobot::control {

class RelocationAction : public Action {
public:
  using Ptr      = std::shared_ptr<RelocationAction>;
  using ConstPtr = std::shared_ptr<const RelocationAction>;

  struct ActionTarget
  {
    std::string body_name;
    std::vector<int> dofs_controlled;
  };

  struct ContactTarget
  {
    std::string body_name;
    std::vector<int> dofs_controlled;
    math::Vector3d bfrc_desired;
  };

  struct JointControlTarget
  {
    std::string joint_name;
    double joint_angle;
    enum ValueType
    {
      EXACT,
      SCALE,
      OFFSET
    } action_type
        = EXACT;
  };

  RelocationAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::ConstRigidBodyPtr body,
      kdl::DynamicGrasp::ConstPtr grasp_prev,
      kdl::DynamicGrasp::ConstPtr grasp_target,
      const math::Vector6d& xwrench_ext_desired,
      const std::vector<ActionTarget>& relocate_targets,
      const std::vector<JointControlTarget>& joint_targets,
      const std::vector<ContactTarget>& contact_targets,
      double duration);

  std::pair<math::VectorXd, math::VectorXd> get_command(
      double time,
      const math::VectorXd& qpos_actual,
      const math::VectorXd& qpos_cmd,
      const math::VectorXd& qvel_actual = {}) override;

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

  kdl::DynamicGrasp::ConstPtr m_grasp_prev;
  kdl::DynamicGrasp::ConstPtr m_grasp;

  math::Matrix4d m_xform_obj;

  math::Vector6d m_xwrench_ext_desired;

  std::vector<ActionTarget> m_relocate_targets;
  std::vector<JointControlTarget> m_joint_targets;
  std::vector<ContactTarget> m_contact_targets;

  std::vector<planning::PoseTrajectory::ConstPtr> m_tform_trajs;
  control::AdmittanceController::Ptr m_controller;
  std::vector<
      std::pair<control::AdmittanceControlTarget::Ptr, kdl::WorldFramePtr>>
      m_ctrl_targets;

  math::VectorXd m_cfrc_internal_stack;
  std::unordered_map<std::string, std::size_t> m_name_to_index;

  std::vector<std::pair<int, double>> m_qpos_target;
  planning::LinearTrajectory::Ptr m_qpos_traj;
};

} // namespace crobot::control

