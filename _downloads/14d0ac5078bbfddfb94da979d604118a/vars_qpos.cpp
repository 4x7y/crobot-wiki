#include "crobot/ik/vars_qpos.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
QposVariables::QposVariables(
    const kdl::ConstRigidBodyTreePtr& rbtree,
    bool limited,
    const std::string& name)
  : Variable(rbtree->get_qpos_dim(), name),
    m_rbtree(rbtree),
    m_is_limited(limited) {
  m_qpos = m_rbtree->get_home_qpos();
}

//==============================================================================
void QposVariables::set_values_all(const math::VectorXd& x) {
  CROBOT_ASSERT_ERROR(x.size() == m_qpos.size());
  m_qpos = x;
};

//==============================================================================
math::VectorXd QposVariables::get_values_all() const {
  return m_qpos;
};

//==============================================================================
VecBound QposVariables::get_bounds_all() const {
  VecBound bounds(m_qpos.size());

  if (m_is_limited) {
    std::pair<math::VectorXd, math::VectorXd> limits
        = m_rbtree->get_qpos_limits();
    for (int i = 0; i < m_rbtree->get_qpos_dim(); ++i) {
      bounds.at(i) = Bounds(limits.first[i], limits.second[i]);
    }
  }
  else {
    for (int i = 0; i < m_qpos.size(); ++i) {
      bounds.at(i)
          = Bounds(-math::constantsd::two_pi(), math::constantsd::two_pi());
    }
  }

  return bounds;
}

//==============================================================================
void QposVariables::fix_joint(
    const std::string& joint_name, const math::VectorXd& qpos) {
  // Keep new fixed joint information
  kdl::ConstRigidBodyPtr body   = m_rbtree->get_body_by_joint(joint_name);
  std::pair<int, int> qpos_addr = body->get_joint()->get_qpos_addr()[0];
  CROBOT_ASSERT_ERROR(qpos_addr.second == qpos.size(), "qpos size mismatch");
  m_qpos.segment(qpos_addr.first, qpos_addr.second) = qpos;

  fix_vars(qpos_addr, qpos);
}

//==============================================================================
void QposVariables::fix_joint_range(
    const std::pair<int, int>& range, const math::VectorXd& qpos) {
  CROBOT_ASSERT_ERROR(range.second == qpos.size(), "qpos size mismatch");
  fix_vars(range, qpos);
}
