#pragma once

#include "crobot/control/action.hpp"
#include "crobot/control/admittance_controller.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/planning/linear_trajectory.hpp"

namespace crobot::control {

struct ForceAdjTarget
{
  std::string body_name;
  math::Vector3d force_start;
  math::Vector3d force_final;
  std::vector<int> dofs_controlled;

  ForceAdjTarget(
      const std::string& body_name,
      const math::Vector3d& force_start,
      const math::Vector3d& force_final,
      const std::vector<int>& dofs_controlled = {})
    : body_name(body_name),
      force_start(force_start),
      force_final(force_final),
      dofs_controlled(dofs_controlled) {
  }
};

class ForceAdjustmentAction : public Action {
public:
  using Ptr      = std::shared_ptr<ForceAdjustmentAction>;
  using ConstPtr = std::shared_ptr<const ForceAdjustmentAction>;

  ForceAdjustmentAction(
      kdl::ConstRigidBodyTreePtr rbtree,
      kdl::ConstRigidBodyPtr body,
      kdl::Grasp::ConstPtr grasp_prev,
      kdl::Grasp::ConstPtr grasp,
      const std::vector<ForceAdjTarget>& targets,
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
  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body;

  kdl::Grasp::ConstPtr m_grasp_prev;
  kdl::Grasp::ConstPtr m_grasp;
  math::Matrix4d m_xform_obj;

  control::AdmittanceController::Ptr m_controller;
  std::vector<control::AdmittanceControlTarget::Ptr> m_ctrl_targets;
  std::vector<planning::LinearTrajectory::ConstPtr> m_force_trajs;
  std::vector<ForceAdjTarget> m_targets;

  bool m_body_efrc_defined;
  math::VectorXd m_cfrc_internal_stack;

  int m_num_wrench_biasis;

  // body name to the contact index in grasp
  std::unordered_map<std::string, std::size_t> m_name_to_index;
  math::MatrixXd m_mat_grasp_pinv;
  math::MatrixXd m_mat_null_space_mapping;

  math::MatrixXd m_mat_grasp_pinv_prev;
  std::unordered_map<std::string, std::size_t> m_name_to_index_prev;
};

} // namespace crobot::control

