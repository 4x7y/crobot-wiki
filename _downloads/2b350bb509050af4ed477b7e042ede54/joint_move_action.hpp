#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/pose_trajectory.hpp"

namespace crobot::control {

class JointMoveAction : public Action {
public:
  using Ptr      = std::shared_ptr<JointMoveAction>;
  using ConstPtr = std::shared_ptr<const JointMoveAction>;

  struct ContactControlTarget
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

  JointMoveAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::Grasp::ConstPtr grasp_target,
      const math::Matrix4d& xform_obj,
      const std::vector<ContactControlTarget>& grasping_targets,
      const std::vector<JointControlTarget>& joint_targets,
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
  kdl::Grasp::ConstPtr m_grasp;
  math::Matrix4d m_xform_obj;
  AdmittanceController::Ptr m_controller;
  std::vector<ContactControlTarget> m_contact_targets;
  std::vector<JointControlTarget> m_joint_targets;
  std::vector<std::pair<int, double>> m_qpos_target;
  planning::LinearTrajectory::Ptr m_qpos_traj;
  std::vector<std::pair<AdmittanceControlTarget::Ptr, kdl::WorldFramePtr>>
      m_ctrl_targets;
};

} // namespace crobot::control

