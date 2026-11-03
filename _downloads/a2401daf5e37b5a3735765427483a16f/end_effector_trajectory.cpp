#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/planning/end_effector_trajectory.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
EndEffectorTrajectory::EndEffectorTrajectory(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name,
    const math::VectorXd& qpos_start,
    const math::Matrix4d& tform_end,
    std::shared_ptr<const TimeScaling> time_scaling)
  : BodyTrajectory(rbtree->get_body(body_name), time_scaling->get_duration()),
    m_time_scaling(time_scaling) {
  std::vector<math::Matrix4d> tforms = rbtree->forward_kinematics(qpos_start);
  math::Matrix4d tform_start         = get_body()->get_xform(tforms);
  m_pose_traj
      = std::make_shared<PoseTrajectory>(tform_start, tform_end, time_scaling);
  m_qvel_addr = m_body->get_support_qvel_addr();
}

//==============================================================================
math::VectorXd EndEffectorTrajectory::get_position(double time) const {
  (void)time;
  return math::VectorXd();
}

//==============================================================================
math::VectorXd EndEffectorTrajectory::get_velocity(double time) const {
  (void)time;
  return math::VectorXd();
}
