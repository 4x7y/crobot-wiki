#include "crobot/ik/obj_jnt_limit_target.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
ObjJntLimitTarget::ObjJntLimitTarget(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::vector<std::string>& joint_names,
    double weight)
  : ObjTarget(rbtree, weight) {
  auto qpos_limits          = m_rbtree->get_qpos_limits();
  m_qpos_lb                 = qpos_limits.first;
  m_qpos_ub                 = qpos_limits.second;
  math::VectorXd qpos_range = m_qpos_ub - m_qpos_lb;
  m_qpos_range2             = qpos_range.array() * qpos_range.array();
  m_qpos_center             = (m_qpos_ub + m_qpos_lb) / 2.0;

  m_qpos_addrs = math::VectorXd(m_rbtree->get_qpos_dim());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    int id = m_rbtree->get_joint(joint_names[i])->get_qvel_addr()[0].first;
    m_qpos_addrs[id] = 1;
  }
}

//==============================================================================
math::VectorXd ObjJntLimitTarget::get_partial_deriv(
    const math::VectorXd& qpos, const math::VectorXd& qvel) const {
  (void)qvel;
  CROBOT_ASSERT_ERROR(qpos.size() == m_rbtree->get_qpos_dim());

  math::VectorXd deriv = qpos - m_qpos_center;
  deriv                = deriv.array() / m_qpos_range2.array();

  // go in negative direction to move towards the center
  return -get_weight() * (m_qpos_addrs.cwiseProduct(deriv));
}
