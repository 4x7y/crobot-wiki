#include "crobot/planning/rdt_rbtree.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/jacobian_ik_dof_target.hpp"
#include "crobot/ik/jacobian_ik_pose_target.hpp"
#include "crobot/math/matrix.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RdtRbtree::RdtRbtree(
    kdl::ConstRigidBodyTreePtr rbtree,
    const math::VectorXd& qpos,
    const std::string& body_name,
    const math::Matrix4d& tform,
    PoseTrajectory::ConstPtr traj,
    int num_iters,
    double tol)
  : m_rbtree(rbtree),
    m_qpos(qpos),
    m_body(rbtree->get_body(body_name)),
    m_tform(tform),
    m_traj(traj),
    m_num_iters(num_iters),
    m_tol_squared(tol * tol),
    m_ncons(0) {

  m_moving_frame  = std::make_shared<kdl::WorldFrame>();
  m_moving_target = std::make_shared<ik::JacobianIKDofTarget>(
      m_moving_frame,
      m_rbtree->get_body(body_name),
      std::vector<int>{3, 4, 5},
      math::EulerAngleSeq::ZYX,
      tform.tform2trvec(),
      tform.tform2rotm());
  add_target(m_moving_target);
}

//==============================================================================
std::vector<math::VectorXd> RdtRbtree::compute() {
  for (const auto& qpos_fixed : m_qvel_addr_fixed) {
    if (qpos_fixed.second) {
      m_qpos[qpos_fixed.first] = qpos_fixed.second.value();
    }
  }
  math::VectorXd qpos = m_qpos;
  std::vector<math::VectorXd> path;
  path.reserve(m_num_iters + 1);

  std::vector<int> qvel_addr(m_qvel_addr.begin(), m_qvel_addr.end());
  math::VectorXd dx(m_ncons);
  math::MatrixXd jacb_full(m_ncons, m_rbtree->get_qvel_dim());

  double duration = m_traj->get_duration();
  double dt       = duration / m_num_iters;

  for (int i = 0; i <= m_num_iters; i++) {
    double time = dt * i;
    m_moving_frame->set_xform(m_traj->get_tform(time));

    math::VectorXd qs                  = qpos(qvel_addr);
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

    int row = 0;
    for (const ik::JacobianIKTarget::Ptr& target : m_targets) {
      math::VectorXd dxi;
      math::MatrixXd jacbi;
      std::tie(dxi, jacbi) = target->compute_value_and_jacobian(tforms);
      int ndofs            = target->get_ndofs();

      dx.segment(row, ndofs)           = dxi;
      jacb_full.middleRows(row, ndofs) = jacbi;
      row += ndofs;
    }

    math::MatrixXd jacb     = jacb_full(Eigen::all, qvel_addr);
    math::MatrixXd jacb_inv = math::calculate_jacb_inv(jacb, 0.0, false);
    math::VectorXd dq       = jacb_inv * dx;

    qpos(qvel_addr) += dq;
    path.push_back(qpos);
  }

  return path;
}

//==============================================================================
void RdtRbtree::add_target(ik::JacobianIKTarget::Ptr target) {
  m_targets.emplace_back(target);
  m_ncons += target->get_ndofs();

  std::vector<int> qvel_addr = target->get_qvel_addrs();
  for (int addr : qvel_addr) {
    if (m_qvel_addr_fixed.find(addr) == m_qvel_addr_fixed.end()) {
      m_qvel_addr.insert(addr);
    }
  }
}

//==============================================================================
void RdtRbtree::add_target(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    const math::Matrix4d& tform,
    math::EulerAngleSeq euler_seq,
    const std::vector<int>& dofs) {

  ik::JacobianIKTarget::Ptr target = std::make_shared<ik::JacobianIKDofTarget>(
      desired_frame,
      m_rbtree->get_body(body_name),
      dofs,
      euler_seq,
      tform.tform2trvec(),
      tform.tform2rotm());
  add_target(target);
}

//==============================================================================
void RdtRbtree::fix_joint(const std::string& joint_name, double value) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  CROBOT_ASSERT_ERROR(
      qvel_addr.second == 1,
      "Only single-DOF joints can be fixed in this function.");
  m_qvel_addr_fixed.insert({qvel_addr.first, value});
  m_qvel_addr.erase(qvel_addr.first);
}

//==============================================================================
void RdtRbtree::fix_joint(const std::string& joint_name) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  for (int i = 0; i < qvel_addr.second; i++) {
    m_qvel_addr_fixed.insert({i + qvel_addr.first, std::nullopt});
    m_qvel_addr.erase(i + qvel_addr.first);
  }
}

//==============================================================================
void RdtRbtree::fix_body(const std::string& body_name) {
  std::vector<int> qvel_addr
      = m_rbtree->get_body(body_name)->get_support_qvel_addr();
  for (std::size_t i = 0; i < qvel_addr.size(); i++) {
    m_qvel_addr_fixed.insert({qvel_addr[i], std::nullopt});
    m_qvel_addr.erase(qvel_addr[i]);
  }
}
