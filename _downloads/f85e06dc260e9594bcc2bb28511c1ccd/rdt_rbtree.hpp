#pragma once

#include <set>

#include "crobot/ik/jacobian_ik_target.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/planning/pose_trajectory.hpp"

namespace crobot::planning {

class RdtRbtree {
public:
  using Ptr      = std::shared_ptr<RdtRbtree>;
  using ConstPtr = std::shared_ptr<const RdtRbtree>;

  RdtRbtree(
      kdl::ConstRigidBodyTreePtr rbtree,
      const math::VectorXd& qpos,
      const std::string& body_name,
      const math::Matrix4d& tform,
      PoseTrajectory::ConstPtr traj,
      int num_iters,
      double tol = 1e-6);

  std::vector<math::VectorXd> compute();

  void add_target(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const math::Matrix4d& tform,
      math::EulerAngleSeq euler_seq,
      const std::vector<int>& dofs);

  void add_spin_target(
      kdl::ConstFramePtr desired_frame,
      const std::string& body_name,
      const math::Matrix3d& rotm,
      const math::Vector3d& bpos);

  void fix_joint(const std::string& joint_name, double value);

  void fix_joint(const std::string& joint_name);

  void fix_body(const std::string& body_name);

private:
  void add_target(ik::JacobianIKTarget::Ptr target);

  kdl::ConstRigidBodyTreePtr m_rbtree;
  math::VectorXd m_qpos;
  std::optional<math::VectorXd> m_qpos_nref;

  kdl::ConstRigidBodyPtr m_body;
  math::Matrix4d m_tform;
  PoseTrajectory::ConstPtr m_traj;
  ik::JacobianIKTarget::Ptr m_moving_target;
  kdl::WorldFrame::Ptr m_moving_frame;

  std::vector<ik::JacobianIKTarget::Ptr> m_targets;
  int m_num_iters;
  double m_tol_squared;
  int m_ncons;

  std::set<int> m_qvel_addr;
  std::unordered_map<int, std::optional<double>> m_qvel_addr_fixed;
};

} // namespace crobot::planning

