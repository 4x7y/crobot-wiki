#pragma once

#include "crobot/control/action.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/planning/pose_trajectory.hpp"

namespace crobot::control {

// simple joint move action without contact control

class SimpleJointMoveAction : public Action {
public:
  using Ptr      = std::shared_ptr<SimpleJointMoveAction>;
  using ConstPtr = std::shared_ptr<const SimpleJointMoveAction>;

  struct JointControlTarget
  {
    std::string joint_name;
    double joint_angle;
  };

  SimpleJointMoveAction(
      kdl::ConstRigidBodyTreePtr rbtree,
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

  void set_null_space_obj(ik::NullSpaceObj::Ptr obj) override;

  kdl::Grasp::ConstPtr get_grasp() const override;

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;
  std::vector<std::pair<int, double>> m_qpos_target;
  planning::LinearTrajectory::Ptr m_qpos_traj;
};

} // namespace crobot::control

